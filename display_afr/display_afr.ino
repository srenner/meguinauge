#include <LiquidCrystal.h>
#include <math.h>

//RS,EN,DB4,DB5,DB6,DB7
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

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
  lcd.setCursor(0,3);
  lcd.write(byte(0));
  lcd.write(byte(1));
  lcd.write(byte(2));
  lcd.write(byte(3));
  lcd.write(byte(4));
}

void loop() {
  double afr = 13.5;
  double tgt = 14.7;
  
  lcd.setCursor(4, 0);
  lcd.print(afr);
  lcd.setCursor(4, 2);
  lcd.print(tgt);
  
  //lcd.setCursor(16,0);
  //lcd.print("Rich");
  draw_afr_bar(afr, 1);
  draw_afr_bar(tgt, 3);
}

void draw_afr_bar(double value, int row) {
  lcd.setCursor(0, row);
  int bars = round((value - 10.0) / 10.0 * 100);
  int fullBars = bars/5;
  
  int partialBars = bars % 5;
  //lcd.setCursor(14, row);
  //lcd.print(bars);
  lcd.setCursor(0, row);

  for(int i = 0; i < fullBars; i++) {
    lcd.write(byte(4));
  }
  if(partialBars > 0) {
    lcd.write(byte(partialBars - 1));    
  }
  
}

