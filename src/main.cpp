//#include <Arduino.h>
#include <Servo.h>
#include <GyverPower.h>
#include <TimerMs.h>

bool DEBUG = true;

uint32_t sleeptime_ms = 3600000; // 3600000 ms = 1 hour
uint32_t worktime_ms = 10000; //10000 ms = 10 s
int dtime = 50; // dtime=diffirence time
int tol = 100; //tol=tolerance
bool parked = false;

Servo horizontal; // horizontal servo
int servoh = 30;
int servohLimitHigh = 90;
int servohLimitLow = 5;

Servo vertical; // vertical servo
int servov = 60;
int servovLimitHigh = 80;
int servovLimitLow = 30;

// LDR pin connections
// name = analogpin;
int ldrlt = A1; //LDR top left - BOTTOM LEFT <--- BDG
int ldrrt = A0; //LDR top rigt - BOTTOM RIGHT
int ldrld = A4; //LDR down left - TOP LEFT
int ldrrd = A2; //ldr down rigt - TOP RIGHT

TimerMs tmr(worktime_ms, 1, 1);

void setup() {
    if ( DEBUG ) {
       Serial.begin(9600);
    }
    power.hardwareDisable(PWR_I2C | PWR_SPI | PWR_USB);
    power.setSleepMode(POWERDOWN_SLEEP);
    horizontal.attach(9);
    vertical.attach(8);
    horizontal.write(servoh);
    vertical.write(servov);
    delay(2500);
    tmr.start();
}

void loop() {
    int lt = analogRead(ldrlt); // top left
    int rt = analogRead(ldrrt); // top right
    int ld = analogRead(ldrld); // down left
    int rd = analogRead(ldrrd); // down right

    int avt = (lt + rt) / 2; // average value top
    int avd = (ld + rd) / 2; // average value down
    int avl = (lt + ld) / 2; // average value left
    int avr = (rt + rd) / 2; // average value right
    int dvert = avt - avd; // check the diffirence of up and down
    int dhoriz = avl - avr;// check the diffirence og left and rigt
    if ( DEBUG) {
       Serial.print("lt: ");
       Serial.print(lt);
       Serial.print(" || rt: ");
       Serial.print(rt);
       Serial.print(" || ld: ");
       Serial.print(ld);
       Serial.print(" || rd: ");
       Serial.println(rd);
    }

    if ((lt+rt+ld+rd) > (tol*4)) {
       parked = false;
       if (-1*tol > dvert || dvert > tol) {
          if (avt > avd) {
             servov = ++servov;
             if (servov > servovLimitHigh) {
                servov = servovLimitHigh;
             }
       } else if (avt < avd) {
             servov= --servov;
             if (servov < servovLimitLow) {
                servov = servovLimitLow;
             }
       }
       if ( DEBUG ) {
          Serial.print("Vertical Pos: ");
          Serial.print(servov);
          Serial.print(" | Horizontal Pos: ");
          Serial.println(servoh);
       }
       vertical.write(servov);
       }
       if (-1*tol > dhoriz || dhoriz > tol) {// check if the diffirence is in the tolerance else change horizontal angle
          if (avl > avr) {
             servoh = --servoh;
             if (servoh < servohLimitLow) {
                servoh = servohLimitLow;
             }
          } else if (avl < avr) {
             servoh = ++servoh;
             if (servoh > servohLimitHigh) {
                servoh = servohLimitHigh;
             }
          } else if (avl == avr) {
             delay(5000);
          }
          if ( DEBUG ) {
             Serial.print("Vertical Pos: ");
             Serial.print(servov);
             Serial.print(" | Horizontal Pos: ");
             Serial.println(servoh);
          }
          horizontal.write(servoh);
       }
       delay(dtime);
    } else {
       if (parked == false) {
          parked = true;
          tmr.stop();
          if ( DEBUG ) {
             Serial.println("Going to START Position. Prepare for Sunrise...");
          }
          horizontal.write(30);
          vertical.write(30);
          delay(2500);
          if ( DEBUG ) {
             Serial.println("Going to PowerDown sleep for 60 minutes");
          }
          delay(100);
          power.sleepDelay(sleeptime_ms);
          if ( DEBUG ) {
             Serial.println("Waking up for check is Sunrise ?");
          }
          tmr.start();
       } else {
             tmr.stop();
             if ( DEBUG ) {
                Serial.println("It`s on start postion. Waiting for sunrise...");
             }
             if ( DEBUG ) {
                Serial.println("Going to PowerDown sleep for 60 minutes");
             }
             delay(100);
             power.sleepDelay(sleeptime_ms);
             if ( DEBUG ) {
                Serial.println("Waking up for check is Sunrise ?");
             }
             tmr.start();
       }
    }

    if (tmr.tick()) {
       if ( DEBUG ) {
          Serial.println("10 seconds of work");
          Serial.println("Going to PowerDown sleep for 60 minutes");
      }
       delay(100);
       power.sleepDelay(sleeptime_ms);
       if ( DEBUG ) {
             Serial.println("Waking up for 10 seconds to work");
       }
       tmr.start();
   }
}