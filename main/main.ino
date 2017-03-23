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

  //message->id = 0x5ea; //group 1514 //0x5e8;
  //message->header.rtr = 0;
  //message->header.length = 8;
  
  currentMillis = millis();
  if(currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;
    lcd.setCursor(4, 2);
    lcd.print(tgt);
    draw_bar(tgt, 3, 10.0, 20.0);

    lcd.setCursor(4, 0);
    lcd.print(afr);
    draw_bar(afr, 1, 10.0, 20.0);
    
    //Serial.println(tgt);
    //Serial.println((double)badDataCount / (double)goodDataCount);
      
  }

  unsigned char len = 0;
  unsigned char buf[8];


  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        


        if(canId == 1514) {
          Serial.println("-----------------------------");
          Serial.print("Get data from ID: ");
          Serial.println(canId, HEX);
          tgt = (double)buf[0] / 10.0;
          afr = (double)buf[1] / 10.0;
          for(int i = 0; i<2; i++)    // print the data
          {
              Serial.print(buf[i]);
              Serial.print("\t");
          }
          Serial.println();          
        }


  }


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

