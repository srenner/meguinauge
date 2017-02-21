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
}

void loop() {
  double afr = 12.5;
  double tgt = 14.7;
  
  lcd.setCursor(4, 0);
  lcd.print(afr);
  lcd.setCursor(4, 2);
  lcd.print(tgt);
  
  //lcd.setCursor(16,0);
  //lcd.print("Rich");
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

