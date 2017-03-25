#include <mcp_can.h>
#include <mcp_can_dfs.h>


#include <global.h>
#include <LiquidCrystal.h>
#include <math.h>

//RS,EN,DB4,DB5,DB6,DB7
LiquidCrystal lcd(14, 3, 4, 5, 6, 7);

int brightness = 255;

byte fill1[8] = {
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000
};
byte fill2[8] = {
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000
};
byte fill3[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100
};
byte fill4[8] = {
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110
};
byte fill5[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

struct EngineVariable
{
  String shortLabel;
  //String longLabel;
  double currentValue;
  double previousValue;
  double minimum;
  double maximum;
  char unit;
};

EngineVariable engine_map   = {"MAP", 0.0, 0.0, 10.0, 250.0, "KPA"};
EngineVariable calc_vac     = {"VAC", 0.0, 0.0, -200.0, 100.0, "HG"};           
EngineVariable calc_bst     = {"BST", 0.0, 0.0, 0.0, 50.0, "PSI"};       
EngineVariable engine_rpm   = {"RPM", 0.0, 0.0, 0.0, 6500.0, ""};                
EngineVariable engine_clt   = {"CLT", 0.0, 0.0, 160.0, 240.0, ""};               
EngineVariable engine_tps   = {"TPS", 0.0, 0.0, 0.0, 100.0, "%"};   
EngineVariable engine_pw1   = {"PW1", 0.0, 0.0, 0.0, 50.0, "MS"};        
EngineVariable engine_pw2   = {"PW2", 0.0, 0.0, 0.0, 50.0, "MS"};        
EngineVariable engine_iat   = {"IAT", 0.0, 0.0, 20.0, 150.0, ""};     
EngineVariable engine_adv   = {"ADV", 0.0, 0.0, 10.0, 60.0, "DEG"};
EngineVariable engine_tgt   = {"TGT", 0.0, 0.0, 10.0, 20.0, ""};
EngineVariable engine_afr   = {"AFR", 0.0, 0.0, 10.0, 20.0, ""};
EngineVariable engine_ego   = {"EGO", 0.0, 0.0, 0.0, 100.0, "%"};
EngineVariable engine_egt   = {"EGT", 0.0, 0.0, 20.0, 2000.0, ""};
EngineVariable engine_pws   = {"PWS", 0.0, 0.0, 0.0, 50.0, "MS"};
EngineVariable engine_bat   = {"BAT", 0.0, 0.0, 0.0, 20.0, "V"};
EngineVariable engine_sr1   = {"SR1", 0.0, 0.0, 0.0, 10000.0, ""};
EngineVariable engine_sr2   = {"SR2", 0.0, 0.0, 0.0, 10000.0, ""};
EngineVariable engine_knk   = {"KNK", 0.0, 0.0, 0.0, 50.0, "DEG"};
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 300.0, "MPH"};
EngineVariable engine_tcr   = {"TCR", 0.0, 0.0, 0.0, 50.0, "DEG"};
EngineVariable engine_lct   = {"LCT", 0.0, 0.0, 0.0, 100.0, "DEG"};

long startTime;
long endTime;

double afr;
double tgt;

unsigned long interval = 100;
unsigned long lastMillis = 0;
unsigned long currentMillis = 0;

int messageReceived;

unsigned long goodDataCount = 0;
unsigned long badDataCount = 0;

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); 

void setup() {

  lcd.createChar(0, fill1);
  lcd.createChar(1, fill2);
  lcd.createChar(2, fill3);
  lcd.createChar(3, fill4);
  lcd.createChar(4, fill5);
  
  lcd.begin(20, 4);
  lcd.print(engine_clt.shortLabel);
  lcd.setCursor(0, 2);
  lcd.print("TGT");

  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
      {
          Serial.println("CAN BUS Shield init fail");
          Serial.println(" Init CAN BUS Shield again");
          delay(100);
      }
  Serial.println("CAN BUS Shield init ok!");
}

void loop() {
  //Serial.println(engine_clt.shortLabel);
  currentMillis = millis();
  if(currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;
    lcd.setCursor(4, 2);
    lcd.print(engine_tgt.currentValue);
    draw_bar(engine_tgt, 3);

    lcd.setCursor(4, 0);
    lcd.print(engine_clt.currentValue);
    draw_bar(engine_clt, 1);
    
  }

  unsigned char len = 0;
  unsigned char buf[8];


  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        
        switch(canId) {
          case 1512:
            engine_map.previousValue = engine_map.currentValue;
            engine_map.currentValue = ((buf[0] * 256) + buf[1]) / 10.0;

            engine_map.previousValue = engine_map.currentValue;
            engine_rpm.currentValue = buf[2] * 256 + buf[3];

            engine_clt.previousValue = engine_clt.currentValue;
            engine_clt.currentValue = (buf[4] * 256 + buf[5]) / 10;
            
            break;
          case 1513:

            break;
          case 1514:
            engine_tgt.previousValue = engine_tgt.currentValue;
            engine_tgt.currentValue = (double)buf[0] / 10.0;

            engine_afr.previousValue = engine_afr.currentValue;
            engine_afr.currentValue = (double)buf[1] / 10.0;
            break;
          case 1515:

            break;
          case 1516:

            break;
          default:
            //do nothing
            break;
        }
  }

  
}

void draw_bar(EngineVariable engineVar, int row) {


  if(engineVar.currentValue == engineVar.previousValue) {
    //relax homeboy
  }
  else{
    lcd.setCursor(0, row);
    
    //delete previous bar if it is too long
    //todo optimize to only delete the needful
    if(engineVar.currentValue < engineVar.previousValue) {
      lcd.write("                    ");
      lcd.setCursor(0, row);
    }
    
    int bars = ((engineVar.currentValue - engineVar.minimum) * 100) / (engineVar.maximum - engineVar.minimum);
    int fullBars = bars/5;
    int partialBars = bars % 5;
  
    //prevent graph overrun
    if(fullBars >= 20) {
      fullBars = 20;
      partialBars = 0;
    }
    
    lcd.setCursor(0, row);
    for(int i = 0; i < fullBars; i++) {
      lcd.write(byte(4));
    }
    if(partialBars > 0) {
      lcd.write(byte(partialBars - 1));    
    }  
  }

  
}

