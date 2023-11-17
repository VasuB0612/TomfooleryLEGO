#include <LiquidCrystal.h>
#include <Wire.h>

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define Motor_return 0
#define Motor_forward 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

#define CMPS14_address 0x60

int pwm_L = 0;
int pwm_R = 0;
int countL = 0;
int countR = 0;
byte raw;
int rawInit;
int angleValue;
bool state = true;

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

void setup() {
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  Wire.begin();
  lcd.begin(20, 4);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), counterLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(2), counterRight, RISING);
  attachInterrupt(digitalPinToInterrupt(19), setState, RISING);
}

void loop() {
  if (state == true) {
    
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
  float angle = float(raw)/255*360;

  display(angle);
  if (y < -5) {
    digitalWrite(Motor_R_dir_pin,Motor_forward);  
    digitalWrite(Motor_L_dir_pin,Motor_forward); 
    pwm_L = -y*5;
    pwm_R = -y*5;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R); 
  } 
  else if (y > 5) {
    digitalWrite(Motor_R_dir_pin,Motor_return);  
    digitalWrite(Motor_L_dir_pin,Motor_return);
    pwm_L = y*5;
    pwm_R = y*5;
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R); 

  } 
  else if (x > 5) {
    digitalWrite(Motor_R_dir_pin,Motor_forward);  
    digitalWrite(Motor_L_dir_pin,Motor_forward); 
      pwm_R = x*5;
      analogWrite(Motor_R_pwm_pin, pwm_R); 
      analogWrite(Motor_L_pwm_pin, 0);
    } 
    else if (x < -5) {
      digitalWrite(Motor_R_dir_pin,Motor_forward);  
    digitalWrite(Motor_L_dir_pin,Motor_forward); 
      pwm_L = -x*5;
      analogWrite(Motor_R_pwm_pin, 0); 
      analogWrite(Motor_L_pwm_pin, pwm_L);
    }
  else {
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin,pwm_L);
    analogWrite(Motor_R_pwm_pin,pwm_R); 
  }
  }
  else if (state == false) {
  if (Serial.available() > 0) {
    Serial.println("Entered ESP Mode.");
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);
    int print = message.indexOf("print");
    int drive = message.indexOf("Move");
    int turn = message.indexOf("Turn");
    int degree = message.indexOf("Degree");

    if (print > -1) {
      Serial.println("Command = Print");
      print = message.indexOf(":");

      if (print > -1) {
        String stat = message.substring(print + 1);
        Serial.println(stat);
        lcd.clear();
        lcd.setCursor((20 - stat.length()) / 2, 1);
        lcd.print(stat);
      }
    } else if (drive > -1) {
      Serial.println("Command = Move");
      drive = message.indexOf(":");

      if (drive > -1) {
        String stat = message.substring(drive + 1);
        Serial.println(stat);
        int value = stat.toInt();

        if (value > 0) {
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("Driving the car");
          lcd.setCursor(2, 2);
          lcd.print("forwards ");
          lcd.print(value);
          lcd.print(" cm");

          pwm_L = 1000;
          pwm_R = 1000;
          digitalWrite(Motor_R_dir_pin, Motor_forward);
          digitalWrite(Motor_L_dir_pin, Motor_forward);
          analogWrite(Motor_L_pwm_pin, pwm_L);
          analogWrite(Motor_R_pwm_pin, pwm_R);
          delay(value * 50);
        } else if (value < 0) {
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("Driving the car");
          lcd.setCursor(2, 2);
          lcd.print("backwards ");
          lcd.print(-value);
          lcd.print(" cm");

          pwm_L = 1000;
          pwm_R = 1000;
          digitalWrite(Motor_R_dir_pin, Motor_return);
          digitalWrite(Motor_L_dir_pin, Motor_return);
          analogWrite(Motor_L_pwm_pin, pwm_L);
          analogWrite(Motor_R_pwm_pin, pwm_R);
          delay(-value * 50);
        } else {
          Serial.println("0..?");
        }
        pwm_L = 0;
        pwm_R = 0;
        analogWrite(Motor_L_pwm_pin, pwm_L);
        analogWrite(Motor_R_pwm_pin, pwm_R);
      }
    } else if (turn > -1) {
      Serial.println("Command = Turn");
      turn = message.indexOf(":");

      if (turn > -1) {
        String stat = message.substring(turn + 1);
        Serial.println(stat);
        int value = stat.toInt();

        if (value > 0) {
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("Turning the car");
          lcd.setCursor(2, 2);
          lcd.print(value);
          lcd.print(" *");

          int angleFinal = value;

          Wire.beginTransmission(CMPS14_address);
          Wire.write(1);
          Wire.endTransmission(false);
          Wire.requestFrom(CMPS14_address, 1, true);
          if (Wire.available()) {
            raw = Wire.read();
            rawInit = raw;
            angleValue = byteToAngle(raw - rawInit);
          }

          while (angleValue < angleFinal) {
            Wire.beginTransmission(CMPS14_address);
            Wire.write(1);
            Wire.endTransmission(false);
            Wire.requestFrom(CMPS14_address, 1, true);
            if (Wire.available()) {
              raw = Wire.read();
              angleValue = byteToAngle(raw - rawInit);
            }

            pwm_L = 0;
            pwm_R = 1000;
            digitalWrite(Motor_R_dir_pin, Motor_forward);
            digitalWrite(Motor_L_dir_pin, Motor_forward);
            analogWrite(Motor_L_pwm_pin, pwm_L);
            analogWrite(Motor_R_pwm_pin, pwm_R);
          }
        } else if (value < 0) {
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("Turning the car");
          lcd.setCursor(2, 2);
          lcd.print(value);
          lcd.print(" *");

          int angleFinal = 360 + value;

          Wire.beginTransmission(CMPS14_address);
          Wire.write(1);
          Wire.endTransmission(false);
          Wire.requestFrom(CMPS14_address, 1, true);
          if (Wire.available()) {
            raw = Wire.read();
            rawInit = raw;
            angleValue = byteToAngle(raw + 255 - rawInit);
          }

          while (angleValue > angleFinal) {
            Wire.beginTransmission(CMPS14_address);
            Wire.write(1);
            Wire.endTransmission(false);
            Wire.requestFrom(CMPS14_address, 1, true);
            if (Wire.available()) {
              raw = Wire.read();
              angleValue = byteToAngle(raw + 255 - rawInit);
            }

            pwm_L = 1000;
            pwm_R = 0;
            digitalWrite(Motor_R_dir_pin, Motor_forward);
            digitalWrite(Motor_L_dir_pin, Motor_forward);
            analogWrite(Motor_L_pwm_pin, pwm_L);
            analogWrite(Motor_R_pwm_pin, pwm_R);
          }
        } else {
          Serial.println("0..?");
        }
        pwm_L = 0;
        pwm_R = 0;
        analogWrite(Motor_L_pwm_pin, pwm_L);
        analogWrite(Motor_R_pwm_pin, pwm_R);
      }
    } else if (degree > -1) {
      Serial.println("Command = Degree");
      degree = message.indexOf(":");
      if (degree > -1) {
        String stat = message.substring(degree + 1);
        Serial.println(stat);
        int value = stat.toInt();

        if (degree > -1) {
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("Turning the car");
          lcd.setCursor(2, 2);
          lcd.print("to ");
          lcd.print(value);
          lcd.print(" *");

          Wire.beginTransmission(CMPS14_address);
          Wire.write(1);
          Wire.endTransmission(false);
          Wire.requestFrom(CMPS14_address, 1, true);
          if (Wire.available()) {
            raw = Wire.read();
            rawInit = raw;
            angleValue = byteToAngle(raw);
          }

          int angleFinal = angleValue - value;

          if (angleFinal <= -180) {
            angleFinal = -angleFinal;
          } else if (angleFinal > -180 && angleFinal < 180) {
            angleFinal = angleFinal;
          } else if (angleFinal >= 180) {
            angleFinal = angleFinal - 360;
          }
          // if angleFinal is above 0, turn left
          // if angleFinal is below 0, turn right
          if (angleFinal < 0) {
            angleFinal = -angleFinal;

              Wire.beginTransmission(CMPS14_address);
              Wire.write(1);
              Wire.endTransmission(false);
              Wire.requestFrom(CMPS14_address, 1, true);
              if (Wire.available()) {
                raw = Wire.read();
                rawInit = raw;
                angleValue = byteToAngle(raw - rawInit);
              }
            while (angleValue < angleFinal) {
              Wire.beginTransmission(CMPS14_address);
              Wire.write(1);
              Wire.endTransmission(false);
              Wire.requestFrom(CMPS14_address, 1, true);
              if (Wire.available()) {
                raw = Wire.read();
                angleValue = byteToAngle(raw - rawInit);
              }

              pwm_L = 0;
              pwm_R = 1000;
              digitalWrite(Motor_R_dir_pin, Motor_forward);
              digitalWrite(Motor_L_dir_pin, Motor_forward);
              analogWrite(Motor_L_pwm_pin, pwm_L);
              analogWrite(Motor_R_pwm_pin, pwm_R);
            }
          } else if (angleFinal > 0) {
              Wire.beginTransmission(CMPS14_address);
              Wire.write(1);
              Wire.endTransmission(false);
              Wire.requestFrom(CMPS14_address, 1, true);
              if (Wire.available()) {
                raw = Wire.read();
                rawInit = raw;
                angleValue = byteToAngle(raw + 255 - rawInit);
              }

              angleFinal = 360 - angleFinal;
            while (angleValue > angleFinal) {
              Wire.beginTransmission(CMPS14_address);
              Wire.write(1);
              Wire.endTransmission(false);
              Wire.requestFrom(CMPS14_address, 1, true);
              if (Wire.available()) {
                raw = Wire.read();
                angleValue = byteToAngle(raw + 255 - rawInit);
              }

              pwm_L = 1000;
              pwm_R = 0;
              digitalWrite(Motor_R_dir_pin, Motor_forward);
              digitalWrite(Motor_L_dir_pin, Motor_forward);
              analogWrite(Motor_L_pwm_pin, pwm_L);
              analogWrite(Motor_R_pwm_pin, pwm_R);
            }
          }
          pwm_L = 0;
          pwm_R = 0;
          analogWrite(Motor_L_pwm_pin, pwm_L);
          analogWrite(Motor_R_pwm_pin, pwm_R);
        } else {
          Serial.println("WHAAAAT?\n");
            }
      }
    } 
    else {
      Serial.println("No command found\n");
    }
  }
  }
}

int byteToAngle(byte raw) {
  return map(raw, 0, 255, 0, 360);
}

void display(float angle) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Total Count: ");
  lcd.print(countL + countR);
  lcd.setCursor(0, 1);
  lcd.print("Dist Left: ");
  lcd.print(countL/8);
  lcd.print(" cm");
  lcd.setCursor(0, 2);
  lcd.print("Dist Right: ");
  lcd.print(countR/8);
  lcd.print(" cm");
  lcd.setCursor(0, 3);
  lcd.print("Angle: ");
  lcd.print(angle);
  lcd.print(" (");
  lcd.print(cardinals(angle));
  lcd.print(")");
  delay(50);
}

String cardinals(float angle) {
  if (angle <= 67.5 && angle > 22.5) {
    return "NE";
  } 
  else if (angle <= 112.5 && angle > 67.5) {
    return "E";
  } 
    else if (angle <= 157.5 && angle > 112.5) {
    return "SE";
  } 
    else if (angle <= 202.5 && angle > 157.5) {
    return "S";
  } 
    else if (angle <= 247.5 && angle > 202.5) {
    return "SW";
  } 
    else if (angle <= 292.5 && angle > 247.5) {
    return "W";
  } 
    else if (angle <= 337.5 && angle > 292.5) {
    return "NW";
  } 
    else if (angle <= 360 && angle > 337.5) {
    return "N";
  } 
      else if (angle <= 22.5 && angle >= 0) {
    return "N";
  } 
}

void setState() {
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay && state == true) {
    state = false;
    Serial.println("Entered ESP Mode.");
  }
  else if (currentTime - lastDebounceTime > debounceDelay && state == false) {
    state = true;
    Serial.println("Entered Manual Control Mode.");
  }
  lastDebounceTime = currentTime;
}

void counterLeft() {
  countL++;
}
void counterRight() {
  countR++;
}
