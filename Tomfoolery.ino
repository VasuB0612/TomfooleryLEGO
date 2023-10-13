#include <LiquidCrystal.h>
#include <Wire.h>

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

#define CMPS14_address        0x60

int pwm_L = 0;
int pwm_R = 0;
int velocity = 85;
int gear = 0;
int count = 0;
byte raw;

void setup()
{
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  Serial.begin(9600);
  lcd.begin(20, 4);

  attachInterrupt(digitalPinToInterrupt(3), counter, RISING);
  attachInterrupt(digitalPinToInterrupt(19), shifter, RISING);
  Wire.begin();
}

void loop()                    
{
    Wire.beginTransmission(CMPS14_address);
      Wire.write(1);

  Wire.endTransmission(false);

  Wire.requestFrom(CMPS14_address, 1, true);

  if (Wire.available() >= 1 ) {
    raw = Wire.read();
  }

  float xPotValue = -(analogRead(A9)-512)/5.12;
  float yPotValue = (analogRead(A8)-512)/5.12;
  int x = int(round(xPotValue)); 
  int y = int(round(yPotValue));

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("; Y: ");
  Serial.print(y);
  Serial.print("; Count: ");
  Serial.print(count);
  Serial.print("; Angle: ");
  Serial.println(raw);
  display(x, y);
  if (y > 5) {
    digitalWrite(Motor_R_dir_pin,Motor_forward);  
    digitalWrite(Motor_L_dir_pin,Motor_forward); 
    pwm_L = velocity;
    pwm_R = velocity;
    if (x > 5) {
      analogWrite(Motor_R_pwm_pin,0); 
      analogWrite(Motor_L_pwm_pin,pwm_L);
    } else if (x < -5) {
      analogWrite(Motor_R_pwm_pin,pwm_R); 
      analogWrite(Motor_L_pwm_pin,0);
    } else {
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R); 
    }
  } 
  else if (y < -5) {
    digitalWrite(Motor_R_dir_pin,Motor_return);  
    digitalWrite(Motor_L_dir_pin,Motor_return);
    pwm_L = velocity;
    pwm_R = velocity;
    if (x > 5) {
      analogWrite(Motor_R_pwm_pin,pwm_R); 
      analogWrite(Motor_L_pwm_pin,0);
    } else if (x < -5) {
      analogWrite(Motor_R_pwm_pin,0); 
      analogWrite(Motor_L_pwm_pin,pwm_L);
    } else {
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R); 
    }
  } 
  else {
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R); 
  }
}      

void display(int x, int y) {
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Tomfoolery");
  lcd.setCursor(0, 1);
  lcd.print("  Gear:");
  lcd.print(gear);
  lcd.print("  Velo:");
  lcd.print(velocity);
  lcd.setCursor(3, 2);
  lcd.print("X: ");
  lcd.print(x);
  lcd.setCursor(12, 2);
  lcd.print("Y: ");
  lcd.print(y);
  lcd.setCursor(6, 3);
  lcd.print("Angle: ");
  lcd.print(raw);
  delay(50);
}

void shifter() {
  if (gear >= 2) {
    gear = 0;
    velocity = 85;
  }
  else {
    gear++;
    if (gear == 0) {
      velocity = 85;
    }
    else if (gear == 1) {
      velocity = 170;
    }
    else if (gear == 2) {
      velocity = 255;
    }
  }
}

void counter() {
  count++;
}