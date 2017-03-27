#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <LiquidCrystal.h>

//RS,EN,DB4,DB5,DB6,DB7
LiquidCrystal lcd(14, 3, 4, 5, 6, 7);

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
  int decimalPlaces;
};

EngineVariable engine_map   = {"MAP", 0.0, 0.0, 10.0, 250.0, 1};    //manifold absolute pressure
EngineVariable calc_vac     = {"VAC", 0.0, 0.0, -200.0, 100.0, 1};  //vacuum  
EngineVariable calc_bst     = {"BST", 0.0, 0.0, 0.0, 50.0, 1};      //boost 
EngineVariable engine_rpm   = {"RPM", 0.0, 0.0, 0.0, 6500.0, 0};    //engine rpm
EngineVariable engine_clt   = {"CLT", 0.0, 0.0, 160.0, 240.0, 0};   //coolant temp
EngineVariable engine_tps   = {"TPS", 0.0, 0.0, 0.0, 100.0, 0};     //throttle position
EngineVariable engine_pw1   = {"PW1", 0.0, 0.0, 0.0, 20.0, 3};      //injector pulse width bank 1
EngineVariable engine_pw2   = {"PW2", 0.0, 0.0, 0.0, 20.0, 3};      //injector pulse width bank 2
EngineVariable engine_iat   = {"IAT", 0.0, 0.0, 40.0, 125.0, 0};    //intake air temp aka 'mat'
EngineVariable engine_adv   = {"ADV", 0.0, 0.0, 10.0, 40.0, 1};     //ignition advance
EngineVariable engine_tgt   = {"TGT", 0.0, 0.0, 10.0, 20.0, 1};     //afr target
EngineVariable engine_afr   = {"AFR", 0.0, 0.0, 10.0, 20.0, 1};     //air fuel ratio
EngineVariable engine_ego   = {"EGO", 0.0, 0.0, 70.0, 130.0, 0};    //ego correction %
EngineVariable engine_egt   = {"EGT", 0.0, 0.0, 100.0, 2000.0, 0};  //exhaust gas temp
EngineVariable engine_pws   = {"PWS", 0.0, 0.0, 0.0, 20.0, 3};      //injector pulse width sequential
EngineVariable engine_bat   = {"BAT", 0.0, 0.0, 0.0, 20.0, 1};      //battery voltage
EngineVariable engine_sr1   = {"SR1", 0.0, 0.0, 0.0, 10000.0, 3};   //generic sensor 1
EngineVariable engine_sr2   = {"SR2", 0.0, 0.0, 0.0, 10000.0, 3};   //generic sensor 2
EngineVariable engine_knk   = {"KNK", 0.0, 0.0, 0.0, 50.0, 1};      //knock ignition retard
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 300.0, 0};     //vehicle speed
EngineVariable engine_tcr   = {"TCR", 0.0, 0.0, 0.0, 50.0, 1};      //traction control ignition retard
EngineVariable engine_lct   = {"LCT", 0.0, 0.0, 0.0, 100.0, 1};     //launch control timing

unsigned long interval = 100;
unsigned long lastMillis = 0;
unsigned long currentMillis = 0;

unsigned long goodDataCount = 0;
unsigned long badDataCount = 0;

const int SPI_CS_PIN = 10;

EngineVariable* idleGauges[8];
bool idleModeReady = false;

enum gaugeMode {
  idle,
  two,
  four
};

MCP_CAN CAN(SPI_CS_PIN); 

void setup() {

  lcd.createChar(0, fill1);
  lcd.createChar(1, fill2);
  lcd.createChar(2, fill3);
  lcd.createChar(3, fill4);
  lcd.createChar(4, fill5);

  idleGauges[0] = &engine_rpm;
  idleGauges[1] = &engine_map;
  idleGauges[2] = &engine_afr;
  idleGauges[3] = &engine_adv;
  idleGauges[4] = &engine_clt;
  idleGauges[5] = &engine_iat;
  idleGauges[6] = &engine_pw1;
  idleGauges[7] = &engine_bat;
  
  lcd.begin(20, 4);
  //lcd.print(engine_rpm.shortLabel);
  //lcd.setCursor(0, 2);
  //lcd.print(engine_afr.shortLabel);

  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
      {
          Serial.println("CAN BUS Shield init fail");
          Serial.println(" Init CAN BUS Shield again");
          delay(100);
      }
  Serial.println("COMM OK");
}

void loop() {
  load_from_can();
  
  currentMillis = millis();
  if(currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;
    if(engine_rpm.currentValue < 2000) {
      draw_idle_gauges();      
    }
    else {
      if(idleModeReady) {
        lcd.clear();
        idleModeReady = false;
      }
      if(engine_rpm.currentValue != engine_rpm.previousValue) {
        lcd.setCursor(4, 0);
        if(is_current_value_shorter(engine_rpm)) {
          lcd.print("     ");
          lcd.setCursor(4, 0);
        }
        lcd.print(engine_rpm.currentValue, engine_rpm.decimalPlaces);
        draw_bar(engine_rpm, 1);
      }
      if(engine_afr.currentValue != engine_afr.previousValue) {
        lcd.setCursor(4, 2);
        lcd.print(engine_afr.currentValue);
        draw_bar(engine_afr, 3);      
      }      
    }
  }
}

void start_idle_mode() {
    lcd.clear();

    //left column of labels
    lcd.setCursor(0, 0);
    lcd.print(idleGauges[0]->shortLabel);
    lcd.setCursor(0, 1);
    lcd.print(idleGauges[1]->shortLabel);
    lcd.setCursor(0, 2);
    lcd.print(idleGauges[2]->shortLabel);
    lcd.setCursor(0, 3);
    lcd.print(idleGauges[3]->shortLabel);

    //draw dividing line
    lcd.setCursor(9, 0);
    lcd.write(byte(4));
    lcd.setCursor(9, 1);
    lcd.write(byte(4));
    lcd.setCursor(9, 2);
    lcd.write(byte(4));
    lcd.setCursor(9, 3);
    lcd.write(byte(4));

    //right column of labels
    lcd.setCursor(11, 0);
    lcd.print(idleGauges[4]->shortLabel);
    lcd.setCursor(11, 1);
    lcd.print(idleGauges[5]->shortLabel);
    lcd.setCursor(11, 2);
    lcd.print(idleGauges[6]->shortLabel);
    lcd.setCursor(11, 3);
    lcd.print(idleGauges[7]->shortLabel);

    idleModeReady = true;
}

void draw_idle_gauges() {

  if(!idleModeReady) {
    start_idle_mode();
  }
  
  lcd.setCursor(4, 0);
  lcd.print(idleGauges[0]->currentValue, idleGauges[0]->decimalPlaces);

  lcd.setCursor(4, 1);
  lcd.print(idleGauges[1]->currentValue, idleGauges[1]->decimalPlaces);

  lcd.setCursor(4, 2);
  lcd.print(idleGauges[2]->currentValue, idleGauges[2]->decimalPlaces);

  lcd.setCursor(4, 3);
  lcd.print(idleGauges[3]->currentValue, idleGauges[3]->decimalPlaces);

  lcd.setCursor(15, 0);
  lcd.print(idleGauges[4]->currentValue, idleGauges[4]->decimalPlaces);

  lcd.setCursor(15, 1);
  lcd.print(idleGauges[5]->currentValue, idleGauges[5]->decimalPlaces);

  lcd.setCursor(15, 2);
  lcd.print(idleGauges[6]->currentValue, idleGauges[6]->decimalPlaces);

  lcd.setCursor(15, 3);
  lcd.print(idleGauges[7]->currentValue, idleGauges[7]->decimalPlaces);
}

void load_from_can() {
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

        engine_rpm.previousValue = engine_rpm.currentValue;
        //engine_rpm.currentValue = buf[2] * 256 + buf[3];
        //round rpm to nearest 10
        engine_rpm.currentValue = round((buf[2] * 256 + buf[3]) / 10) * 10;

        engine_clt.previousValue = engine_clt.currentValue;
        engine_clt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;

        engine_tps.previousValue = engine_tps.currentValue;
        engine_tps.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        
        break;
      case 1513:
        engine_pw1.previousValue = engine_pw1.currentValue;
        engine_pw1.currentValue = (buf[0] * 256 + buf[1]) / 1000.0;

        engine_pw2.previousValue = engine_pw2.currentValue;
        engine_pw2.currentValue = (buf[2] * 256 + buf[3]) / 1000.0;

        engine_iat.previousValue = engine_iat.currentValue;
        engine_iat.currentValue = (buf[4] * 256 + buf[5]) / 10.0;

        engine_adv.previousValue = engine_adv.currentValue;
        engine_adv.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        
        break;
      case 1514:
        engine_tgt.previousValue = engine_tgt.currentValue;
        engine_tgt.currentValue = (double)buf[0] / 10.0;

        engine_afr.previousValue = engine_afr.currentValue;
        engine_afr.currentValue = (double)buf[1] / 10.0;

        engine_ego.previousValue = engine_ego.currentValue;
        engine_ego.currentValue = (buf[2] * 256 + buf[3]) / 10.0;

        engine_egt.previousValue = engine_egt.currentValue;
        engine_egt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;

        engine_pws.previousValue = engine_pws.currentValue;
        engine_pws.currentValue = (buf[6] * 256 + buf[7]) / 1000.0;
        
        break;
      case 1515:
        engine_bat.previousValue = engine_bat.currentValue;
        engine_bat.currentValue = (buf[0] * 256 + buf[1]) / 10.0;

        //not tested
        engine_sr1.previousValue = engine_sr1.currentValue;
        engine_sr1.currentValue = (buf[2] * 256 + buf[3]) / 10.0;

        //not tested
        engine_sr2.previousValue = engine_sr2.currentValue;
        engine_sr2.currentValue = (buf[4] * 256 + buf[5]) / 10.0;

        //not tested
        engine_knk.previousValue = engine_knk.currentValue;
        engine_knk.currentValue = (buf[6] * 256) / 10.0;

        break;
      case 1516:
        //not tested
        engine_vss.previousValue = engine_vss.currentValue;
        engine_vss.currentValue = (buf[0] * 256 + buf[1]) / 10.0;

        //not tested
        engine_tcr.previousValue = engine_tcr.currentValue;
        engine_tcr.currentValue = (buf[2] * 256 + buf[3]) / 10.0;

        engine_lct.previousValue = engine_lct.currentValue;
        engine_lct.previousValue = (buf[4] * 256 + buf[5]) / 10.0;
        
        break;
      default:
        //do nothing
        break;
    }
  }
}

//note: using log10 would work but this is faster
bool is_current_value_shorter(EngineVariable engine) {
  int currentLength;
  if(engine.currentValue >= 10000) {
    currentLength = 5;
  }
  else if(engine.currentValue >= 1000) {
    currentLength = 4;
  }
  else if(engine.currentValue >= 100) {
    currentLength = 3;
  }
  else if(engine.currentValue >= 10) {
    currentLength = 2;
  }
  else {
    currentLength = 1;
  }
  if(engine.decimalPlaces > 0) {
    currentLength++;
    currentLength+= engine.decimalPlaces;
  }

  int previousLength;
  if(engine.previousValue >= 10000) {
    previousLength = 5;
  }
  else if(engine.previousValue >= 1000) {
    previousLength = 4;
  }
  else if(engine.previousValue >= 100) {
    previousLength = 3;
  }
  else if(engine.previousValue >= 10) {
    previousLength = 2;
  }
  else {
    previousLength = 1;
  }
  if(engine.decimalPlaces > 0) {
    previousLength++;
    previousLength += engine.decimalPlaces;
  }

  return currentLength < previousLength;
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

