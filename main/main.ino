#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
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

long startTime;
long endTime;

double afr;
double tgt;

unsigned long interval = 100;
unsigned long lastMillis = 0;
unsigned long currentMillis = 0;

tCAN message;
int messageReceived;

tCAN emtpyMessage;

unsigned long goodDataCount = 0;
unsigned long badDataCount = 0;

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
  int can_init = mcp2515_init(1); //CANSPEED_500 = 1
}

void loop() {

  ecu_req(&message);
  
  currentMillis = millis();

  if(messageReceived == 1) {
    if(currentMillis - lastMillis >= interval) {
      lastMillis = currentMillis;
      lcd.setCursor(4, 2);
      lcd.print(tgt);
      //Serial.println(tgt);
      Serial.println((double)badDataCount / (double)goodDataCount);
      
    }
    messageReceived = 0;    
  }

  
  

  
}

void draw_bar(double value, int row, double minimum, double maximum) {
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

void ecu_req(tCAN *message) 
{
  message->id = 0x5ea; //group 1514 //0x5e8;
  message->header.rtr = 0;
  message->header.length = 8;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  //Serial.println("trying to get data");
  if(mcp2515_check_message()) {
    
    //Serial.print("there is a message ");
    mcp2515_get_message(message);
    
    if(message->data[0] >= 100) {
        tgt = (double)message->data[0] / 10.0;
        //Serial.print("good data ");
        //Serial.println(tgt);
        goodDataCount++;
    }
    else {
      ///Serial.println("bad data ");
      badDataCount++;
    }
    
    messageReceived = 1;
  }
  else {
    messageReceived = 0;
    //Serial.println("no data");
  }

}

