#include <LiquidCrystal.h>
#include <LIDARLite.h>
#include <Wire.h>

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

LIDARLite lidar;

#define Motor_return 0
#define Motor_forward 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

#define CMPS14_address 0x60

int pwm_L = 0; // pwm value for left wheel
int pwm_R = 0; // pwm value for right wheel
int countL = 0; // count of pulses of left wheel
int countR = 0; // count of pulses of right wheel
byte raw; // current angle of compass in bytes
int rawInit; // initial angle of compass in bytes
int angleValue; // current angle of compass
bool state = false; // current mode;
int lidarDist;

float a;
float b;
float c;
float area = 0;
float volume = 0;

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

// GROUP NAME: Tomfoolery

void setup() {
  pinMode(A8, INPUT);
    pinMode(A9, INPUT);

    Wire.begin();
    lcd.begin(20, 4);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(3), counterLeft, RISING); // counts pulses of the left wheel
    attachInterrupt(digitalPinToInterrupt(2), counterRight, RISING); // counts pulses of the right wheel
    attachInterrupt(digitalPinToInterrupt(19), setState, RISING); // changes mode between manual and ESP; false = manual, true = ESP

    lidar.begin(0, true);
    lidar.configure(0);

}

int variable = 1;

void loop() {
// read current angle of compass
        Wire.beginTransmission(CMPS14_address);
        Wire.write(1);
        Wire.endTransmission(false);
        Wire.requestFrom(CMPS14_address, 1, true);
        if (Wire.available() >= 1) {
            raw = Wire.read();
        }

        int x = int(round(-(analogRead(A9) - 512) / 5.12)); // x axis of joystick
        int y = int(round((analogRead(A8) - 512) / 5.12)); // y axis of joystick
        float angle = float(raw) / 255 * 360; // angle of compass
        lidarDist = lidar.distance(true);

        displayManual(angle, lidarDist);
        driveManual(x, y);
        delay(33);

        if (state) {
          if (variable == 1) {
            a = lidar.distance();
            turnESP();
            variable = 2;
          }
          else if (variable == 2) {
            b = lidar.distance();
            turnESP();
            variable = 3;
          }
          else if (variable == 3) {
            lcd.print("Turn the car up cuh.")
            c = lidar.distance();
            turnESP();
            variable = 1;
          }
          state = false;
        }
        if (a && b) {
          area = a*b;
        }
        if (a && b && c) {
          volume = a*b*c;
        }
}

// responsible for driving the car manually
// x = left joystick, y = right joystick
void driveManual(int x, int y) {
    // drive forward
    if (y < -5) {
        digitalWrite(Motor_R_dir_pin,Motor_forward);
        digitalWrite(Motor_L_dir_pin,Motor_forward);
        pwm_L = -y*5;
        pwm_R = -y*5;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
    }
    // drive backward
    else if (y > 5) {
        digitalWrite(Motor_R_dir_pin,Motor_return);
        digitalWrite(Motor_L_dir_pin,Motor_return);
        pwm_L = y*5;
        pwm_R = y*5;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
    }
    // turn left
    else if (x > 5) {
        digitalWrite(Motor_R_dir_pin,Motor_forward);
        digitalWrite(Motor_L_dir_pin,Motor_forward);
        pwm_R = x*5;
        analogWrite(Motor_R_pwm_pin, pwm_R);
        analogWrite(Motor_L_pwm_pin, 0);
    }
    // turn right
    else if (x < -5) {
        digitalWrite(Motor_R_dir_pin,Motor_forward);
        digitalWrite(Motor_L_dir_pin,Motor_forward);
        pwm_L = -x*5;
        analogWrite(Motor_R_pwm_pin, 0);
        analogWrite(Motor_L_pwm_pin, pwm_L);
    }
    // stop
    else {
        pwm_L = 0;
        pwm_R = 0;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
    }
}

// displays the current state of the car in manual mode
void displayManual(float angle, int lidarDist) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("A: ");
    lcd.print(a);
    lcd.setCursor(10, 0);
    lcd.print("B: ");
    lcd.print(b);
    lcd.setCursor(0, 1);    
    lcd.print("C: ");
    lcd.print(c);
    lcd.setCursor(0, 2);
    lcd.print("Area: ");
    lcd.print(area);
    lcd.setCursor(0, 3);
    lcd.print("Volume: ");
    lcd.print(volume);
    delay(50);
}

void turnESP(int value) {
    // if value is above 0, turn right
    if (value > 0) {
        lcd.print(value);
        lcd.print(" degrees right");
        int angleFinal = value;

        // read current angle of compass
        Wire.beginTransmission(CMPS14_address);
        Wire.write(1);
        Wire.endTransmission(false);
        Wire.requestFrom(CMPS14_address, 1, true);
        if (Wire.available()) {
            raw = Wire.read();
            rawInit = raw;
            angleValue = byteToAngle(raw - rawInit);
        }

        // angleValue defaults to 0, and turn right until angleValue >= angleFinal
        while (angleValue < angleFinal) {
            // read current angle of compass
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
    }
    // if value is below 0, turn left
    else if (value < 0) {
        lcd.print(-value);
        lcd.print(" degrees left");

        int angleFinal = 360 + value;

        // read current angle of compass
        Wire.beginTransmission(CMPS14_address);
        Wire.write(1);
        Wire.endTransmission(false);
        Wire.requestFrom(CMPS14_address, 1, true);
        if (Wire.available()) {
            raw = Wire.read();
            rawInit = raw;
            angleValue = byteToAngle(raw + 255 - rawInit);
        }

        // angleValue defaults to 360, and turn left until angleValue <= angleFinal
        while (angleValue > angleFinal) {
            // read current angle of compass
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
    else {
        Serial.println("0..?");
    }
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
}

// converts angle to cardinal direction
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
    // ESP Mode
    if (currentTime - lastDebounceTime > debounceDelay && state) {
        state = false;

        Serial.println("State: false");      
    }
    // Manual Mode
    else if (currentTime - lastDebounceTime > debounceDelay && !state) {
        state = true;
        
        Serial.println("State: true");
    }

    lastDebounceTime = currentTime;
}

// converts byte to angle
int byteToAngle(byte raw) {
    return map(raw, 0, 255, 0, 360);
}

void counterLeft() {
    countL++;
}
void counterRight() {
    countR++;
}