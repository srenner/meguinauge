#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <LiquidCrystal.h>
#include <math.h>

//RS,EN,DB4,DB5,DB6,DB7
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

#define REDLITE 3 
#define GREENLITE 5 
#define BLUELITE 6

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


char can_buffer[64];
char can_data;

//possible parameters to display
double  engine_afr;         //AFR
double  engine_afr_tgt;     //AFR target
double  engine_map;         //MAP sensor
double  engine_boost;       //boost (psi) calculated from map value
double  engine_adv;         //ignition advance
int     engine_rpm;         //RPM
int     engine_tps;         //throttle position
double  engine_vbat;        //battery voltage
int     engine_clt;         //coolant temp
int     engine_iat;         //intake air temp



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

  pinMode(REDLITE, OUTPUT); 
  pinMode(GREENLITE, OUTPUT); 
  pinMode(BLUELITE, OUTPUT);
  
  setBacklight(255, 0, 0);

  Serial.begin(9600);

  //Initialize CAN Controller 
  if(Canbus.init(CANSPEED_500))  /* Initialize MCP2515 CAN controller at the specified speed */
  {
    Serial.println("CAN Init Ok!");
    //delay(1500);
  }
  else {
    Serial.println("Can't init CAN");
  }

  
}

void loop() {

  ecu_req(can_buffer);
  //Serial.print("buffer: ");
  //Serial.println(can_buffer);
  //Canbus.ecu_req(ENGINE_RPM,buffer);
  //Canbus.message_rx(buffer);
  
  //engine_rpm = buffer;
  //Serial.print("Engine RPM: "); //Uncomment for Serial debugging
  //Serial.println("buffer: " + buffer);
  delay(1500);

  
  double afr = 12.5;
  double tgt = 14.7;
  
  lcd.setCursor(4, 0);
  lcd.print(afr);
  lcd.setCursor(4, 2);
  lcd.print(tgt);
  
  draw_bar(afr, 1, 10, 20);
  draw_bar(tgt, 3, 10, 20);
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

void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest! 
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
  r = map(r, 0, 255, 0, brightness); 
  g = map(g, 0, 255, 0, brightness); 
  b = map(b, 0, 255, 0, brightness);
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  analogWrite(REDLITE, r); 
  analogWrite(GREENLITE, g); 
  analogWrite(BLUELITE, b);
}

char ecu_req(char *buffer) 
{
  tCAN message;
  float engine_data;
  int timeout = 0;
  char message_ok = 0;
  // Prepair message
  message.id = PID_REQUEST;
  message.header.rtr = 0;
  message.header.length = 8;
  message.data[0] = 0x02;
  message.data[1] = 0x01;
  message.data[2] = "all";
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;           
  

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  
  if (mcp2515_send_message(&message)) {
    Serial.println("message sent");
  }
  else {
    Serial.println("message not sent");
  }
        if (mcp2515_check_message()) 
        {
          Serial.println("after check message");
          if (mcp2515_get_message(&message)) 
          {
                  engine_data =  ((message.data[3]*256) + message.data[4])/4;
                  sprintf(buffer,"%d rpm ",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);
              
                  engine_data =  message.data[3] - 40;
                  sprintf(buffer,"%d degC",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);
                                
                  engine_data =  message.data[3];
                  sprintf(buffer,"%d km ",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);

                  engine_data =  ((message.data[3]*256) + message.data[4])/100;
                  sprintf(buffer,"%d g/s",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);
                  
                  engine_data = message.data[3]*0.005;
                  sprintf(buffer,"%d V",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);
                  
                  engine_data = (message.data[3]*100)/255;
                  sprintf(buffer,"%d %% ",(int) engine_data);
                  Serial.print("buffer 1: ");
                  Serial.println(buffer);
                  
            return buffer;
          }
          else {
            Serial.println("nothing to get");
          }
        }
        else {
          Serial.println("check/else");
        }

}
