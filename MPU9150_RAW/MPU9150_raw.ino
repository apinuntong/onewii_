// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
  I2Cdev device library code is placed under the MIT license

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Kalman.h>
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN PB12
bool blinkState = false;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
int16_t tempRaw;

float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
double dT;
void RE_MPU();
void pwmWritefast(uint8 pin, uint16 duty_cycle) ;
float limit(float input, int min_limit, int max_limit);
void myPID();
void PWMControl(int set_motor1, int set_motor2);
int led_out = PB0;
HardwareTimer pwmtimer(3);

float kp = 1000.00f, ki = 0.0, kd = 100.00;
float r_angle, f_angle, output_pid, f_angle_b;
float errSum, dErr, error, lastErr;
float LOutput, ROutput;

//uint64_t mucro_time = 0;
int button1 ;
int buttonnum = 0 ;
int buttonnum2 = 0 ;
double f_angle_R = 0;
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  delay(500);
  Wire.begin();
  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setPeriod(33);
  pinMode(led_out, PWM);
  pwmWritefast(led_out, 0); //1000;
  pinMode(PB5, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(PB3, INPUT_PULLUP);
  //Serial.begin(115200);
  accelgyro.initialize();
  pinMode(LED_PIN, OUTPUT);
}
uint64_t previousMillis = 0;
boolean zxv = 0;
float pitch1 = 0;
float pitchAcc1 = 0;
void loop() {
  // read raw accel/gyro measurements from device
  uint64_t currentMillis = micros();
  if (currentMillis - previousMillis >= 2000) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    RE_MPU();
    //f_angle_b=smooth(0.1f,kalAngleY,f_angle_b);
    f_angle_b = compAngleY + 2.00;
   // Serial.println(f_angle_b);
    if (f_angle_b < 1 && f_angle_b > -1 && buttonnum == 1) {
      buttonnum2 = 1;
    }
    if (f_angle_b < 15 && f_angle_b > -15 && buttonnum2 == 1 && digitalRead(PB3) == 0)
    {
      digitalWrite(PB4, 1);

      myPID();
      float a = 0.1f;
      if (f_angle_b > a || f_angle_b < -a) {
        PWMControl( LOutput,  ROutput);
      } else {
        PWMControl( 0,  0);
      }
      //       Serial.print(LOutput);Serial.print(" ");Serial.println(f_angle_b);
    } else {
      buttonnum2 = 0;
      digitalWrite(PB4, 0);
      errSum = 0;
      PWMControl( 0,  0);
    }
    zxv = !zxv;
    digitalWrite(LED_PIN, zxv );
  }
  if (button1 == 1 && digitalRead(PB3) == 0) {
    buttonnum = 1;
  }
  button1 = digitalRead(PB3);


}
float x;
void RE_MPU() {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //x=smooth(0.01f,gy,x);
  long squaresum = (long)ay * ay + (long)az * az;
  pitch1 += ((-(gy-6.0) / 32.8f) * 0.002f);
  float pitchAcc2 = atan(ax / sqrt(squaresum)) * RAD_TO_DEG;
  pitchAcc1 = smooth(0.05f,pitchAcc2,pitchAcc1);
  if(digitalRead(PB3)==0){
    pitch1 = 1.0f * pitch1 + (1.0f - 1.0f) * pitchAcc1;
  }else{
    pitch1 = 0.95f * pitch1 + (1.0f - 0.95f) * pitchAcc1;
  }
  
  compAngleY = -pitch1;
  
  //  Serial.println(compAngleY);
}
void pwmWritefast(uint8 pin, uint16 duty_cycle) {
  timer_set_compare(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, duty_cycle);
}
void PWMControl(int set_motor1, int set_motor2)
{
  if (set_motor1 > 0) {
    digitalWrite(PB5, 0);
    pwmWritefast(led_out, limit(set_motor1, 0, 2000));
    //Write_PWM_Pin(2, 0);
  } else if (set_motor1 < 0) {
    pwmWritefast(led_out, limit(-set_motor1, 0, 2000));
    digitalWrite(PB5, 1);
    //Write_PWM_Pin(2, 0);
  } else {
    pwmWritefast(led_out, 0);
    digitalWrite(PB5, 0);
    digitalWrite(PB4, 0);
  }


}
void myPID()
{
  lastErr = error;
  error = f_angle_b;
  dErr = (error - lastErr) * 10.0f;
  errSum = limit(errSum + (error * 0.01f), -100, 100);

  output_pid = (kp * error) + (ki * errSum) + (kd * dErr);

  LOutput = output_pid;
  ROutput = output_pid;

}
float limit(float input, int min_limit, int max_limit)
{
  if (input > max_limit)input = max_limit;
  if (input < min_limit)input = min_limit;
  return input;
}

float smooth(float alfa, float new_data, float old_data)
{
  return (old_data + alfa * (new_data - old_data));
}
