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
  char shortLabel;
  char longLabel;
  double currentValue;
  double previousValue;
  double minimum;
  double maximum;
  char unit;
};

EngineVariable engine_map   = {"MAP", "MAP", 0.0, 0.0, 10.0, 250.0, "KPA"};
EngineVariable calc_vac     = {"VAC", "VACUUM", 0.0, 0.0, -200.0, 100.0, "HG"};           
EngineVariable calc_bst     = {"BST", "BOOST", 0.0, 0.0, 0.0, 50.0, "PSI"};       
EngineVariable engine_rpm   = {"RPM", "RPM", 0.0, 0.0, 0.0, 6500.0, ""};                
EngineVariable engine_clt   = {"CLT", "TEMP", 0.0, 0.0, 20.0, 250.0, ""};               
EngineVariable engine_tps   = {"TPS", "THROTTLE", 0.0, 0.0, 0.0, 100.0, "%"};   
EngineVariable engine_pw1   = {"PW1", "PULSE WIDTH 1", 0.0, 0.0, 0.0, 50.0, "MS"};        
EngineVariable engine_pw2   = {"PW2", "PULSE WIDTH 2", 0.0, 0.0, 0.0, 50.0, "MS"};        
EngineVariable engine_iat   = {"IAT", "AIR TEMP", 0.0, 0.0, 20.0, 150.0, ""};     
EngineVariable engine_adv   = {"ADV", "ADVANCE", 0.0, 0.0, 10.0, 60.0, "DEG"};
EngineVariable engine_tgt   = {"TGT", "AIR FUEL TARGET", 0.0, 0.0, 10.0, 20.0, ""};
EngineVariable engine_afr   = {"AFR", "AIR FUEL RATIO", 0.0, 0.0, 10.0, 20.0, ""};
EngineVariable engine_ego   = {"EGO", "EGO CORRECTION", 0.0, 0.0, 0.0, 100.0, "%"};
EngineVariable engine_egt   = {"EGT", "EXHAUST TEMP", 0.0, 0.0, 20.0, 2000.0, ""};
EngineVariable engine_pws   = {"PWS", "PULSE WIDTH SEQ", 0.0, 0.0, 0.0, 50.0, "MS"};
EngineVariable engine_bat   = {"BAT", "BATTERY", 0.0, 0.0, 0.0, 20.0, "V"};
EngineVariable engine_sr1   = {"SR1", "SENSOR 1", 0.0, 0.0, 0.0, 10000.0, ""};
EngineVariable engine_sr2   = {"SR2", "SENSOR 2", 0.0, 0.0, 0.0, 10000.0, ""};
EngineVariable engine_knk   = {"KNK", "KNOCK RTD", 0.0, 0.0, 0.0, 50.0, "DEG"};
EngineVariable engine_vss   = {"VSS", "SPEED", 0.0, 0.0, 0.0, 300.0, "MPH"};
EngineVariable engine_tcr   = {"TCR", "TRACTION RTD", 0.0, 0.0, 0.0, 50.0, "DEG"};
EngineVariable engine_lct   = {"LCT", "LAUNCH CTRL", 0.0, 0.0, 0.0, 100.0, "DEG"};

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
  lcd.print("AFR");
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

  currentMillis = millis();
  if(currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;
    lcd.setCursor(4, 2);
    lcd.print(engine_tgt.currentValue);
    draw_bar(engine_tgt.currentValue, 3, 10.0, 20.0);

    lcd.setCursor(4, 0);
    lcd.print(engine_afr.currentValue);
    draw_bar(engine_afr.currentValue, 1, 10.0, 20.0);
    
  }

  unsigned char len = 0;
  unsigned char buf[8];


  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        
        switch(canId) {
          case 1512:

            break;
          case 1513:

            break;
          case 1514:
            engine_tgt.currentValue = (double)buf[0] / 10.0;
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

void draw_bar(double value, int row, double minimum, double maximum) {
  lcd.setCursor(0, row);
  lcd.write("                    ");
  lcd.setCursor(0, row);
  int bars = ((value - minimum) * 100) / (maximum - minimum);
  int fullBars = bars/5;
  int partialBars = bars % 5;
  lcd.setCursor(0, row);
  for(int i = 0; i < fullBars; i++) {
    lcd.write(byte(4));
  }
  if(partialBars > 0) {
    lcd.write(byte(partialBars - 1));    
  }
}

