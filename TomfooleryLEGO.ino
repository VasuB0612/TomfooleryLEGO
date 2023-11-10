#include <LiquidCrystal.h>
#include <Wire.h>

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define Motor_return          0
#define Motor_forward         1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

#define CMPS14_address        0x60

int pwm_L = 0;
int pwm_R = 0;
int count = 0;
byte raw;
int rawInit;
int angleValue;

void setup() {
    pinMode(A8, INPUT);
    pinMode(A9, INPUT);

    Wire.begin();
    lcd.begin(20, 4);
    Serial.begin(9600);
    Serial.println("Write something to the serial monitor.");
}

void loop() {
    if (Serial.available() > 0) {
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
                        Serial.println(angleValue);
                    }
                    Serial.print("reached ");
                    Serial.println(angleFinal);
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
                        Serial.println(angleValue);
                    }
                    Serial.print("reached ");
                    Serial.println(angleFinal);
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
                String stat = message.substring(turn + 1);
                Serial.println(stat);
                int value = stat.toInt();

                if (value > 0) {
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

                    if (angleFinal < -180) {
                        angleFinal = -angleFinal;
                    }
                    else if (angleFinal >= -180 && angleFinal < 180) {
                        angleFinal = angleFinal;
                    }
                    else if (angleFinal >= 180) {
                        angleFinal = angleFinal - 360;
                    }

                    // if angleFinal is above 0, turn left
                    // if angleFinal is below 0, turn right
                    if (angleFinal < 0) {
                        angleFinal = -angleFinal;

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
                            Serial.println(angleValue);
                        }
                        Serial.print("reached ");
                        Serial.println(angleFinal);
                    }
                    else if (angleFinal > 0) {
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
                            Serial.println(angleValue);
                        }
                        Serial.print("reached ");
                        Serial.println(angleFinal);
                    }
                    pwm_L = 0;
                    pwm_R = 0;
                    analogWrite(Motor_L_pwm_pin, pwm_L);
                    analogWrite(Motor_R_pwm_pin, pwm_R);
                }
                else {
                    Serial.println("WHAAAAT?\n");
                }
            }
        }
        else {
            Serial.println("No command found\n");
        }
    }
}

int byteToAngle(byte raw) {
    return map(raw, 0, 255, 0, 360);
}
