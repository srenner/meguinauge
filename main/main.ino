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
  char* shortLabel;
  float currentValue;
  float previousValue;
  float minimum;
  float maximum;
  byte decimalPlaces;
  unsigned long goodCount;
  unsigned long lowCount;
  unsigned long highCount;
};

EngineVariable engine_map   = {"MAP", 0.0, 0.0, 15.0, 250.0, 1, 0, 0};    //manifold absolute pressure
EngineVariable engine_rpm   = {"RPM", 0.0, 0.0, 0.0, 6500.0, 0, 0, 0};    //engine rpm
EngineVariable engine_clt   = {"CLT", 0.0, 0.0, 20.0, 240.0, 0, 0, 0};    //coolant temp
EngineVariable engine_tps   = {"TPS", 0.0, 0.0, 0.0, 100.0, 0, 0, 0};     //throttle position
EngineVariable engine_pw1   = {"PW1", 0.0, 0.0, 0.0, 20.0, 3, 0, 0};      //injector pulse width bank 1
EngineVariable engine_pw2   = {"PW2", 0.0, 0.0, 0.0, 20.0, 3, 0, 0};      //injector pulse width bank 2
EngineVariable engine_iat   = {"IAT", 0.0, 0.0, 40.0, 150.0, 0, 0, 0};    //intake air temp aka 'mat'
EngineVariable engine_adv   = {"ADV", 0.0, 0.0, 10.0, 40.0, 1, 0, 0};     //ignition advance
EngineVariable engine_tgt   = {"TGT", 0.0, 0.0, 10.0, 20.0, 1, 0, 0};     //afr target
EngineVariable engine_afr   = {"AFR", 0.0, 0.0, 10.0, 20.0, 1, 0, 0};     //air fuel ratio
EngineVariable engine_ego   = {"EGO", 0.0, 0.0, 70.0, 130.0, 0, 0, 0};    //ego correction %
EngineVariable engine_egt   = {"EGT", 0.0, 0.0, 0.0, 2000.0, 0, 0, 0};  //exhaust gas temp
EngineVariable engine_pws   = {"PWS", 0.0, 0.0, 0.0, 20.0, 3, 0, 0};      //injector pulse width sequential
EngineVariable engine_bat   = {"BAT", 0.0, 0.0, 8.0, 18.0, 1, 0, 0};      //battery voltage
EngineVariable engine_sr1   = {"SR1", 0.0, 0.0, 0.0, 999.0, 1, 0, 0};     //generic sensor 1
EngineVariable engine_sr2   = {"SR2", 0.0, 0.0, 0.0, 999.0, 1, 0, 0};     //generic sensor 2
EngineVariable engine_knk   = {"KNK", 0.0, 0.0, 0.0, 50.0, 1, 0, 0};      //knock ignition retard
EngineVariable engine_vss   = {"VSS", 0.0, 0.0, 0.0, 160.0, 0, 0, 0};     //vehicle speed
EngineVariable engine_tcr   = {"TCR", 0.0, 0.0, 0.0, 50.0, 1, 0, 0};      //traction control ignition retard
EngineVariable engine_lct   = {"LCT", 0.0, 0.0, 0.0, 50.0, 1, 0, 0};      //launch control timing

EngineVariable* allGauges[20];

unsigned long currentMillis = 0;

byte displayInterval = 100;
unsigned long lastDisplayMillis = 0;

unsigned int diagnosticInterval = 5000;
unsigned long lastDiagnosticMillis = 0;

const int SPI_CS_PIN = 10;

EngineVariable* dualModeGauges[3][2];
EngineVariable* quadModeGauges[2][4];
EngineVariable* octoModeGauges[1][8];

bool dualModeReady = false;
bool quadModeReady = false;
bool octoModeReady = false;

bool dualModeIndex = 0;
bool quadModeIndex = 0;
bool octoModeIndex = 0;

MCP_CAN CAN(SPI_CS_PIN); 

void setup() {

  lcd.createChar(0, fill1);
  lcd.createChar(1, fill2);
  lcd.createChar(2, fill3);
  lcd.createChar(3, fill4);
  lcd.createChar(4, fill5);
  
  octoModeGauges[0][0] = &engine_rpm;
  octoModeGauges[0][1] = &engine_map;
  octoModeGauges[0][2] = &engine_afr;
  octoModeGauges[0][3] = &engine_adv;
  octoModeGauges[0][4] = &engine_clt;
  octoModeGauges[0][5] = &engine_iat;
  octoModeGauges[0][6] = &engine_pw1;
  octoModeGauges[0][7] = &engine_bat;

  dualModeGauges[0][0] = &engine_rpm;
  dualModeGauges[0][1] = &engine_afr;

  quadModeGauges[0][0] = &engine_rpm;
  quadModeGauges[0][1] = &engine_map;
  quadModeGauges[0][2] = &engine_afr;
  quadModeGauges[0][3] = &engine_tps;
  
  allGauges[0] = &engine_map;
  allGauges[1] = &engine_rpm;
  allGauges[2] = &engine_clt;
  allGauges[3] = &engine_tps;
  allGauges[4] = &engine_pw1;
  allGauges[5] = &engine_pw2;
  allGauges[6] = &engine_iat;
  allGauges[7] = &engine_adv;
  allGauges[8] = &engine_tgt;
  allGauges[9] = &engine_afr;
  allGauges[10] = &engine_ego;
  allGauges[11] = &engine_egt;
  allGauges[12] = &engine_pws;
  allGauges[13] = &engine_bat;
  allGauges[14] = &engine_sr1;
  allGauges[15] = &engine_sr2;
  allGauges[16] = &engine_knk;
  allGauges[17] = &engine_vss;
  allGauges[18] = &engine_tcr;
  allGauges[19] = &engine_lct;
  
  lcd.begin(20, 4);

  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println(F("CAN bus init fail"));
    delay(100);
  }
  Serial.println(F("COMM OK"));

  lcd.clear();
  
}

void loop() {
  load_from_can();
  
  currentMillis = millis();
  if(currentMillis - lastDisplayMillis >= displayInterval && currentMillis > 500) {
    lastDisplayMillis = currentMillis;
      //draw_octo_gauges();
      //draw_dual_gauges();
      draw_quad_gauges();

      //unsigned long badCount = engine_rpm.lowCount + engine_rpm.highCount;
      //unsigned long totalCount = badCount + engine_rpm.goodCount;
      //byte percent = badCount * 100 / totalCount;
      //Serial.println(percent);
  }

  if(currentMillis - lastDiagnosticMillis >= diagnosticInterval && currentMillis > 500) {
    lastDiagnosticMillis = currentMillis;
    Serial.println(calculate_error_light());
  }
}


void clear_mode() {
  dualModeReady = false;
  quadModeReady = false;
  octoModeReady = false;
  lcd.clear();
}

void draw_dual_gauges() {
  if(!dualModeReady) {
    clear_mode();
    lcd.setCursor(0, 0);
    lcd.print(dualModeGauges[0][0]->shortLabel);
    lcd.setCursor(0, 2);
    lcd.print(dualModeGauges[0][1]->shortLabel);
    dualModeReady = true;
  }
  //if(dualModeGauges[0][0]->currentValue != dualModeGauges[0][0]->previousValue) {
    lcd.setCursor(4, 0);
    if(is_current_value_shorter(*dualModeGauges[0][0])) {
      lcd.print(F("     "));
      lcd.setCursor(4, 0);
    }
    lcd.print(dualModeGauges[0][0]->currentValue, dualModeGauges[0][0]->decimalPlaces);
    draw_bar(*dualModeGauges[0][0], 1, 0);
  //}
  //if(dualModeGauges[0][1]->currentValue != dualModeGauges[0][1]->previousValue) {
    lcd.setCursor(4, 2);
    if(is_current_value_shorter(*dualModeGauges[0][1])) {
      lcd.print(F("     "));
      lcd.setCursor(4, 2);
    }
    lcd.print(dualModeGauges[0][1]->currentValue, dualModeGauges[0][1]->decimalPlaces);
    draw_bar(*dualModeGauges[0][1], 3, 0);
  //}
}

void draw_quad_gauges() {
  if(!quadModeReady) {
    clear_mode();
    lcd.setCursor(0, 0);
    lcd.print(quadModeGauges[0][0]->shortLabel);
    lcd.setCursor(0, 1);
    lcd.print(quadModeGauges[0][1]->shortLabel);
    lcd.setCursor(0, 2);
    lcd.print(quadModeGauges[0][2]->shortLabel);
    lcd.setCursor(0, 3);
    lcd.print(quadModeGauges[0][3]->shortLabel);
    quadModeReady = true;
  }

  lcd.setCursor(4, 0);
  if(is_current_value_shorter(*quadModeGauges[0][0])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 0);
  }
  lcd.print(quadModeGauges[0][0]->currentValue, quadModeGauges[0][0]->decimalPlaces);
  draw_bar(*quadModeGauges[0][0], 0, 9);

  lcd.setCursor(4, 1);
  if(is_current_value_shorter(*quadModeGauges[0][1])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 1);
  }
  lcd.print(quadModeGauges[0][1]->currentValue, quadModeGauges[0][1]->decimalPlaces);
  draw_bar(*quadModeGauges[0][1], 1, 9);

  lcd.setCursor(4, 2);
  if(is_current_value_shorter(*quadModeGauges[0][2])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 2);
  }
  lcd.print(quadModeGauges[0][2]->currentValue, quadModeGauges[0][2]->decimalPlaces);
  draw_bar(*quadModeGauges[0][2], 2, 9);

  lcd.setCursor(4, 3);
  if(is_current_value_shorter(*quadModeGauges[0][3])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 3);
  }
  lcd.print(quadModeGauges[0][3]->currentValue, quadModeGauges[0][3]->decimalPlaces);
  draw_bar(*quadModeGauges[0][3], 3, 9);
}

void draw_octo_gauges() {

  if(!octoModeReady) {
    clear_mode();
    //left column of labels
    lcd.setCursor(0, 0);
    lcd.print(octoModeGauges[0][0]->shortLabel);
    lcd.setCursor(0, 1);
    lcd.print(octoModeGauges[0][1]->shortLabel);
    lcd.setCursor(0, 2);
    lcd.print(octoModeGauges[0][2]->shortLabel);
    lcd.setCursor(0, 3);
    lcd.print(octoModeGauges[0][3]->shortLabel);

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
    lcd.print(octoModeGauges[0][4]->shortLabel);
    lcd.setCursor(11, 1);
    lcd.print(octoModeGauges[0][5]->shortLabel);
    lcd.setCursor(11, 2);
    lcd.print(octoModeGauges[0][6]->shortLabel);
    lcd.setCursor(11, 3);
    lcd.print(octoModeGauges[0][7]->shortLabel);

    octoModeReady = true;
  }
  
  lcd.setCursor(4, 0);
  if(is_current_value_shorter(*octoModeGauges[0][0])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 0);
  }
  lcd.print(octoModeGauges[0][0]->currentValue, octoModeGauges[0][0]->decimalPlaces);

  lcd.setCursor(4, 1);
  if(is_current_value_shorter(*octoModeGauges[0][1])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 1);
  }
  lcd.print(octoModeGauges[0][1]->currentValue, octoModeGauges[0][1]->decimalPlaces);

  lcd.setCursor(4, 2);
  if(is_current_value_shorter(*octoModeGauges[0][2])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 2);
  }
  lcd.print(octoModeGauges[0][2]->currentValue, octoModeGauges[0][2]->decimalPlaces);

  lcd.setCursor(4, 3);
  if(is_current_value_shorter(*octoModeGauges[0][3])) {
    lcd.print(F("     "));
    lcd.setCursor(4, 3);
  }
  lcd.print(octoModeGauges[0][3]->currentValue, octoModeGauges[0][3]->decimalPlaces);

  lcd.setCursor(15, 0);
  if(is_current_value_shorter(*octoModeGauges[0][4])) {
    lcd.print(F("     "));
    lcd.setCursor(15, 0);
  }
  lcd.print(octoModeGauges[0][4]->currentValue, octoModeGauges[0][4]->decimalPlaces);

  lcd.setCursor(15, 1);
  if(is_current_value_shorter(*octoModeGauges[0][5])) {
    lcd.print(F("     "));
    lcd.setCursor(15, 1);
  }
  lcd.print(octoModeGauges[0][5]->currentValue, octoModeGauges[0][5]->decimalPlaces);

  lcd.setCursor(15, 2);
  if(is_current_value_shorter(*octoModeGauges[0][6])) {
    lcd.print(F("     "));
    lcd.setCursor(15, 2);
  }
  lcd.print(octoModeGauges[0][6]->currentValue, octoModeGauges[0][6]->decimalPlaces);

  lcd.setCursor(15, 3);
  if(is_current_value_shorter(*octoModeGauges[0][7])) {
    lcd.print(F("     "));
    lcd.setCursor(15, 3);
  }
  lcd.print(octoModeGauges[0][7]->currentValue, octoModeGauges[0][7]->decimalPlaces);
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
        increment_counter(&engine_map);

        engine_rpm.previousValue = engine_rpm.currentValue;
        //engine_rpm.currentValue = buf[2] * 256 + buf[3];
        //round rpm to nearest 10
        engine_rpm.currentValue = round((buf[2] * 256 + buf[3]) / 10.0) * 10.0;
        increment_counter(&engine_rpm);
        
        engine_clt.previousValue = engine_clt.currentValue;
        engine_clt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_clt);
        
        engine_tps.previousValue = engine_tps.currentValue;
        engine_tps.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        increment_counter(&engine_tps);
        
        break;
      case 1513:
        engine_pw1.previousValue = engine_pw1.currentValue;
        engine_pw1.currentValue = (buf[0] * 256 + buf[1]) / 1000.0;
        increment_counter(&engine_pw1);

        engine_pw2.previousValue = engine_pw2.currentValue;
        engine_pw2.currentValue = (buf[2] * 256 + buf[3]) / 1000.0;
        increment_counter(&engine_pw2);

        engine_iat.previousValue = engine_iat.currentValue;
        engine_iat.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_iat);

        engine_adv.previousValue = engine_adv.currentValue;
        engine_adv.currentValue = (buf[6] * 256 + buf[7]) / 10.0;
        increment_counter(&engine_adv);
        
        break;
      case 1514:
        engine_tgt.previousValue = engine_tgt.currentValue;
        engine_tgt.currentValue = (double)buf[0] / 10.0;
        increment_counter(&engine_tgt);

        engine_afr.previousValue = engine_afr.currentValue;
        engine_afr.currentValue = (double)buf[1] / 10.0;
        increment_counter(&engine_afr);

        engine_ego.previousValue = engine_ego.currentValue;
        engine_ego.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_ego);

        engine_egt.previousValue = engine_egt.currentValue;
        engine_egt.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_egt);

        engine_pws.previousValue = engine_pws.currentValue;
        engine_pws.currentValue = (buf[6] * 256 + buf[7]) / 1000.0;
        increment_counter(&engine_pws);
        
        break;
      case 1515:
        engine_bat.previousValue = engine_bat.currentValue;
        engine_bat.currentValue = (buf[0] * 256 + buf[1]) / 10.0;
        increment_counter(&engine_bat);

        //not tested
        engine_sr1.previousValue = engine_sr1.currentValue;
        engine_sr1.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_sr1);

        //not tested
        engine_sr2.previousValue = engine_sr2.currentValue;
        engine_sr2.currentValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_sr2);

        //not tested
        engine_knk.previousValue = engine_knk.currentValue;
        engine_knk.currentValue = (buf[6] * 256) / 10.0;
        increment_counter(&engine_knk);

        break;
      case 1516:
        //not tested
        engine_vss.previousValue = engine_vss.currentValue;
        engine_vss.currentValue = (buf[0] * 256 + buf[1]) / 10.0;
        increment_counter(&engine_vss);

        //not tested
        engine_tcr.previousValue = engine_tcr.currentValue;
        engine_tcr.currentValue = (buf[2] * 256 + buf[3]) / 10.0;
        increment_counter(&engine_tcr);

        engine_lct.previousValue = engine_lct.currentValue;
        engine_lct.previousValue = (buf[4] * 256 + buf[5]) / 10.0;
        increment_counter(&engine_lct);
        
        break;
      default:
        //do nothing
        break;
    }
  }
}

void increment_counter(EngineVariable* engine) {
  if(engine->currentValue > engine->maximum) {
    engine->highCount++;
  }
  else if(engine->currentValue < engine->minimum) {
    engine->lowCount++;
  }
  else {
    engine->goodCount++;
  }  
}

//note: using log10 would work but this is faster
bool is_current_value_shorter(EngineVariable engine) {
  int roundedCurrent = round(engine.currentValue);
  int roundedPrevious = round(engine.previousValue);
  byte currentLength;
  if(roundedCurrent >= 10000) {
    currentLength = 5;
  }
  else if(roundedCurrent >= 1000) {
    currentLength = 4;
  }
  else if(roundedCurrent >= 100) {
    currentLength = 3;
  }
  else if(roundedCurrent >= 10) {
    currentLength = 2;
  }
  else {
    currentLength = 1;
  }
  if(engine.decimalPlaces > 0) {
    currentLength++;
    currentLength+= engine.decimalPlaces;
  }

  byte previousLength;
  if(roundedPrevious >= 10000) {
    previousLength = 5;
  }
  else if(roundedPrevious >= 1000) {
    previousLength = 4;
  }
  else if(roundedPrevious >= 100) {
    previousLength = 3;
  }
  else if(roundedPrevious >= 10) {
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

void draw_bar(EngineVariable engineVar, byte row, byte column) {
  /*if(engineVar.currentValue == engineVar.previousValue) {
    //relax homeboy
  }*/
  if(true){
    lcd.setCursor(column, row);
    
    //delete previous bar if it is too long
    //todo optimize to only delete the needful
    if(engineVar.currentValue < engineVar.previousValue) {

      /*char *blank = malloc((20 - column) + 1);
      memset(blank, ' ', (20 - column));
      blank[(20 - column)] = '\0';
      lcd.write(blank);
      for(byte i = 0; i < (20 - column) + 1; i++) {
        free(blank[i]);
      }
      free(blank);*/

      /* HAVING MEMORY/PERFORMANCE ISSUES HERE. HARDCODING THIS SEEMS TO HELP. */

      if(column == 0) {
        lcd.write("                    ");  //hard coded value for dual mode
      }
      else {
        lcd.write("           ");           //hard-coded value for quad mode
      }
      lcd.setCursor(column, row);
    }

    //calculate bars
    //todo optimize and correct partial bars
    byte percent = ((engineVar.currentValue - engineVar.minimum) * 100) / (engineVar.maximum - engineVar.minimum);
    float bars = (percent/(100.0/(20.0 - column)) * 10.0) / 10.0;
    byte fullBars = (byte)bars;
    byte partialBars = (byte)((bars - ((byte)(bars))) /0.2);

    //todo optimize/fix this
    if(engineVar.currentValue == engineVar.minimum) {
      fullBars = 0;
      partialBars = 0;
    }

    //prevent graph overrun
    if(fullBars >= (20 - column)) {
      fullBars = (20 - column);
      partialBars = 0;
    }
    
    lcd.setCursor(column, row);
    for(int i = 0; i < fullBars; i++) {
      lcd.write(byte(4));
    }
    if(partialBars > 0) {
      lcd.write(byte(partialBars - 1));    
    }  
  }  
}

bool calculate_error_light() {
  byte len = 20; //sizeof(allGauges);
  unsigned long badCount;
  unsigned long totalCount;
  byte percent;
  
  for(byte i = 0; i < len; i++) {
    badCount = allGauges[i]->lowCount + allGauges[i]->highCount;
    totalCount = badCount + allGauges[i]->goodCount;
    percent = badCount * 100 / totalCount;
    if(percent > 9) {
      Serial.println(allGauges[i]->shortLabel);
      Serial.println(allGauges[i]->lowCount);
      Serial.println(allGauges[i]->highCount);
      Serial.println(allGauges[i]->goodCount);
      return true;
    }
  }
  return false;
}

