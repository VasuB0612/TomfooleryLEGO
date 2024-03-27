#include <LiquidCrystal.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <DFRobot_TCS34725.h>

DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS,TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

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
bool state = true; // current mode; true = manual, false = ESP
int lidarDist;

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

// GROUP NAME: Tomfoolery

void setup() {
    pinMode(A8, INPUT);
    pinMode(A9, INPUT);

    Wire.begin();
    lcd.begin(20, 4);
    Serial.begin(9600);
    Serial2.begin(9600);
    attachInterrupt(digitalPinToInterrupt(3), counterLeft, RISING); // counts pulses of the left wheel
    attachInterrupt(digitalPinToInterrupt(2), counterRight, RISING); // counts pulses of the right wheel
    attachInterrupt(digitalPinToInterrupt(19), setState, RISING); // changes mode between manual and ESP; false = manual, true = ESP

    Serial.println("Color View Test!");
    lidar.begin(0, true);
    lidar.configure(0);

    while(!tcs.begin())
    {
      Serial.println("No TCS34725 found ... check your connections");
      delay(1000);
    }
}

void loop() {
    // Manual Mode
    if (state) {
        // read current angle of compass
        Wire.beginTransmission(CMPS14_address);
        Wire.write(1);
        Wire.endTransmission(false);
        Wire.requestFrom(CMPS14_address, 1, true);
        if (Wire.available() >= 1) {
            raw = Wire.read();
            angleValue = byteToAngle(raw);

        }

        int x = int(round(-(analogRead(A9) - 512) / 5.12)); // x axis of joystick
        int y = int(round((analogRead(A8) - 512) / 5.12)); // y axis of joystick
        float angle = float(raw) / 255 * 360; // angle of compass
        lidarDist = lidar.distance(true);

        Serial.print("Lid=");
        Serial.print(lidarDist);
        Serial.print(",Com=");
        Serial.print(angleValue);

        uint16_t clear, red, green, blue;
        tcs.getRGBC(&red, &green, &blue, &clear);
        // turn off LED
        tcs.lock();  
        Serial.print(",R="); Serial.print(red); 
        Serial.print(",G="); Serial.print(green); 
        Serial.print(",B="); Serial.print(blue); 
        Serial.println("\t");


        displayManual(angle);
        driveManual(x, y);
        delay(33);
    }
    // ESP Mode
    else if (!state) {
        displayESP();
  // read current angle of compass
          Wire.beginTransmission(CMPS14_address);
          Wire.write(1);
          Wire.endTransmission(false);
          Wire.requestFrom(CMPS14_address, 1, true);
          if (Wire.available() >= 1) {
              raw = Wire.read();
              angleValue = byteToAngle(raw);

          }
              lidarDist = lidar.distance(true);

          Serial.print("Lid=");
          Serial.print(lidarDist);
          Serial.print(",Com=");
          Serial.println(angleValue);
        if (Serial.available() > 0) {
            String message = Serial.readStringUntil('\n');
            Serial.print("Message received, content: ");
            Serial.println(message);



            int drive = message.indexOf("Move");
            int turn = message.indexOf("Turn");
            int degree = message.indexOf("Degree");
            int track = message.indexOf("Track");
            int mazeDrive = message.indexOf("MazeDrive");

            // if message contains "Move", execute driveESP()
            if (drive > -1) {
                Serial.println("Command = Move");
                drive = message.indexOf(":");

                if (drive > -1) {
                    String stat = message.substring(drive + 1);
                    Serial.println(stat);
                    int value = stat.toInt();

                    driveESP(value);
                }
            }
            // if message contains "Turn", execute turnESP()
            else if (turn > -1) {
                Serial.println("Command = Turn");
                turn = message.indexOf(":");

                if (turn > -1) {
                    String stat = message.substring(turn + 1);
                    Serial.println(stat);
                    int value = stat.toInt();

                    turnESP(value);
                }
            }
            // if message contains "Degree", execute turnToESP()
            else if (degree > -1) {
                Serial.println("Command = Degree");
                degree = message.indexOf(":");
                if (degree > -1) {
                    String stat = message.substring(degree + 1);
                    Serial.println(stat);
                    int value = stat.toInt();

                    turnToESP(value);
                }
            }
            else if (track > -1) {
                Serial.println("Command = Track");
                if (track > -1) {
                  trackWalls();
                }
            } 
            else if (mazeDrive > -1) {
                Serial.println("Command = MazeDrive");
                if (mazeDrive > -1) {

                    while (message.indexOf("MazeStop") < 0) {
                        message = Serial.readStringUntil('\n');
                        // read current angle of compass
                        Wire.beginTransmission(CMPS14_address);
                        Wire.write(1);
                        Wire.endTransmission(false);
                        Wire.requestFrom(CMPS14_address, 1, true);
                        if (Wire.available() >= 1) {
                            raw = Wire.read();
                            angleValue = byteToAngle(raw);

                        }

                        lidarDist = lidar.distance(true);

                        Serial.print("Lid=");
                        Serial.print(lidarDist);
                        Serial.print(",Com=");
                        Serial.print(angleValue);

                        uint16_t clear, red, green, blue;
                        tcs.getRGBC(&red, &green, &blue, &clear);
                        // turn off LED
                        tcs.lock();  
                        Serial.print(",R="); Serial.print(red); 
                        Serial.print(",G="); Serial.print(green); 
                        Serial.print(",B="); Serial.print(blue); 
                        Serial.println("\t");

                        delay(33);
                        digitalWrite(Motor_R_dir_pin,Motor_forward);
                        digitalWrite(Motor_L_dir_pin,Motor_forward); 
                        pwm_L = 100;
                        pwm_R = 100;
                        analogWrite(Motor_L_pwm_pin,pwm_L);
                        analogWrite(Motor_R_pwm_pin,pwm_R);

                        if (blue >= 260 && blue <= 285) {
                          pwm_L = 0;
                          pwm_R = 0;
                          analogWrite(Motor_L_pwm_pin,pwm_L);
                          analogWrite(Motor_R_pwm_pin,pwm_R);
                          turnESP(90);
                          tcs.getRGBC(&red, &green, &blue, &cleasr);
                          delay(33);
                        }
                        else if (red >= 200) {
                          pw
                          _L = 0;
                          pwm_R = 0;
                          analogWrite(Motor_L_pwm_pin,pwm_L);
                          analogWrite(Motor_R_pwm_pin,pwm_R);
                          turnESP(-90);
                          tcs.getRGBC(&red, &green, &blue, &clear);
                          delay(33);
                        }
                        else if (green >= 200) {
                          message = "MazeStop";
                        }
                    }
                    pwm_L = 0;
                    pwm_R = 0;
                    analogWrite(Motor_L_pwm_pin,pwm_L);
                    analogWrite(Motor_R_pwm_pin,pwm_R);
                }
            } else {
                Serial.println("No command found\n");
            }
        }
    }
}

void trackWalls() {
    lidarDist = lidar.distance(true);
    
    digitalWrite(Motor_R_dir_pin,Motor_forward);
    digitalWrite(Motor_L_dir_pin,Motor_forward);

    while (!state) {
      lidarDist = lidar.distance(true);
      if (lidarDist < 10) {
        pwm_L = 0;
        pwm_R = 0;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
      }
      else if (lidarDist <= 35 && lidarDist >= 10) {
        pwm_L = 150;
        pwm_R = 150;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
      }
      else if (lidarDist > 35) {
        pwm_L = 500;
        pwm_R = 500;
        analogWrite(Motor_L_pwm_pin,pwm_L);
        analogWrite(Motor_R_pwm_pin,pwm_R);
      }
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

// responsible for driving the car in ESP mode
// distance = distance to drive in cm
void driveESP(int distance) {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("ESP Mode");
    lcd.setCursor(2, 2);
    lcd.print("Driving the car");
    lcd.setCursor(2, 3);

    lidarDist = lidar.distance(true);
    int turnCount = 0;

    while (lidarDist < 10) {
          lidarDist = lidar.distance(true);
          if (turnCount < 6) {
            turnESP(60);
            turnCount++;
          }
          else if (turnCount == 6) {
            Serial.println("STOP");
          }
    }
    if (lidarDist >= 10) {
    // if distance is above 0, drive forward
    if (distance > 0) {
        lcd.print(distance);
        lcd.print(" cm");
        lcd.print(" forward..");

        int countFinal = countR + distance * 8; // 8 pulses = 1 cm

        // DO NOT REMOVE THESE PRINT STATEMENTS, OTHERWISE THE CODE WILL NOT WORK, I DONT FUCKING KNOW WHY
        Serial.println(countR);
        Serial.println(countFinal);

        while (countR < countFinal) {
            pwm_L = 500;
            pwm_R = 500;
            digitalWrite(Motor_R_dir_pin, Motor_forward);
            digitalWrite(Motor_L_dir_pin, Motor_forward);
            analogWrite(Motor_L_pwm_pin, pwm_L);
            analogWrite(Motor_R_pwm_pin, pwm_R);
        }
    }

    // if distance is below 0, drive backward
    else if (distance < 0) {
        lcd.print(-distance);
        lcd.print(" cm");
        lcd.print(" backward..");

        int countFinal = countR - distance * 8; // 8 pulses = 1 cm

        // DO NOT REMOVE THESE PRINT STATEMENTS, OTHERWISE THE CODE WILL NOT WORK, I DONT FUCKING KNOW WHY
        Serial.println(countR);
        Serial.println(countFinal);

        while (countR < countFinal) {
            pwm_L = 500;
            pwm_R = 500;
            digitalWrite(Motor_R_dir_pin, Motor_return);
            digitalWrite(Motor_L_dir_pin, Motor_return);
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
}

// responsible for turning the car in ESP mode
// value = degrees to turn
void turnESP(int value) {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("ESP Mode");
    lcd.setCursor(2, 2);
    lcd.print("Turning the car");
    lcd.setCursor(2, 3);

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
            pwm_R = 500;
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

            pwm_L = 500;
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

// responsible for turning the car to a specific angle in ESP mode
// value = degrees to turn to
void turnToESP(int value) {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("ESP Mode");
    lcd.setCursor(2, 2);
    lcd.print("Turning the car");
    lcd.setCursor(2, 3);
    lcd.print("to ");
    lcd.print(value);
    lcd.print(" degrees");

    // read current angle of compass
    Wire.beginTransmission(CMPS14_address);
    Wire.write(1);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 1, true);
    if (Wire.available()) {
        raw = Wire.read();
        rawInit = raw;
        angleValue = byteToAngle(raw);
    }

    // calculates which direction is the shortest to turn to
    // angleFinal = angle to turn to
    int angleFinal = angleValue - value;

    if (angleFinal <= -180) {
        angleFinal = -angleFinal;
    }
    else if (angleFinal > -180 && angleFinal < 180) {
        angleFinal = angleFinal;
    }
    else if (angleFinal >= 180) {
        angleFinal = angleFinal - 360;
    }

    // if angleFinal is below 0, turn right
    if (angleFinal < 0) {
        angleFinal = -angleFinal;

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
            Wire.beginTransmission(CMPS14_address);
            Wire.write(1);
            Wire.endTransmission(false);
            Wire.requestFrom(CMPS14_address, 1, true);
            if (Wire.available()) {
                raw = Wire.read();
                angleValue = byteToAngle(raw - rawInit);
            }

            pwm_L = 0;
            pwm_R = 500;
            digitalWrite(Motor_R_dir_pin, Motor_forward);
            digitalWrite(Motor_L_dir_pin, Motor_forward);
            analogWrite(Motor_L_pwm_pin, pwm_L);
            analogWrite(Motor_R_pwm_pin, pwm_R);
        }
    }
    // if angleFinal is above 0, turn left
    else if (angleFinal > 0) {
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

        angleFinal = 360 - angleFinal;

        // angleValue defaults to 360, and turn left until angleValue <= angleFinal
        while (angleValue > angleFinal) {
            Wire.beginTransmission(CMPS14_address);
            Wire.write(1);
            Wire.endTransmission(false);
            Wire.requestFrom(CMPS14_address, 1, true);
            if (Wire.available()) {
                raw = Wire.read();
                angleValue = byteToAngle(raw + 255 - rawInit);
            }

            pwm_L = 500;
            pwm_R = 0;
            digitalWrite(Motor_R_dir_pin, Motor_forward);
            digitalWrite(Motor_L_dir_pin, Motor_forward);
            analogWrite(Motor_L_pwm_pin, pwm_L);
            analogWrite(Motor_R_pwm_pin, pwm_R);
        }
    }
    else {
        Serial.println("WHAAAAT?");
    }
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
}

// converts byte to angle
int byteToAngle(byte raw) {
    return map(raw, 0, 255, 0, 360);
}

// displays the current state of the car in manual mode
void displayManual(float angle) {
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

// displays the current state of the car in ESP mode
void displayESP() {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("ESP Mode");
    lcd.setCursor(0, 1);
    lcd.print("Total Count: ");
    lcd.print(countL + countR);
    lcd.setCursor(0, 2);
    lcd.print("Dist Left: ");
    lcd.print(countL/8);
    lcd.print(" cm");
    lcd.setCursor(0, 3);
    lcd.print("Dist Right: ");
    lcd.print(countR/8);
    lcd.print(" cm");
    delay(50);
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

// changes mode between manual and ESP
void setState() {
    unsigned long currentTime = millis();

    // ESP Mode
    if (currentTime - lastDebounceTime > debounceDelay && state) {
        state = false;
        Serial.println("Entered ESP Mode.");
    }
    // Manual Mode
    else if (currentTime - lastDebounceTime > debounceDelay && !state) {
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