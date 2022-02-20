#include <Servo.h>
#include <GyverPower.h>
#include <TimerMs.h>

bool DEBUG = true;

uint32_t sleeptime_ms = 3600000;//1800000; // 3600000 ms = 1 hour
uint32_t sleeptime_min = sleeptime_ms / 60000; 
uint32_t worktime_ms = 10000; //10000 ms = 10 s
uint32_t worktime_sec = worktime_ms / 1000;
int dtime = 50; // dtime=diffirence time
int tol = 10; //tol=tolerance
bool parked = false;

// ----- Motor`s Definition Section -----------------
Servo horizontal; // Horizontal servo (Up/Down)
int servoh = 40;
int servoh_old = servoh;
int servohLimitHigh = 90; //Up Position Limit
int servohLimitLow = 5;   //Down Position Linit

Servo vertical; // Vertical servo (Left/Right)
int servov = 60;
int servov_old = servov;
int servovLimitHigh = 120; // Right Position Limit
int servovLimitLow = 5;    // Left Position Limit
// ---------------------------------------------------

/* LDR pin connections:
-----------------------
      A3 | A2
     ----+----
      A1 | A0
-----------------------
*/
// name = analogpin;
int ldrlt = A0; //Down Right
int ldrrt = A1; //Down Left
int ldrld = A2; //Up Right
int ldrrd = A3; //Up Letf

TimerMs tmr(worktime_ms, 1, 1);

void servo_pos_print(){
   Serial.print("Vertical Pos: ");
   Serial.print(vertical.read());
   Serial.print(" | Horizontal Pos: ");
   Serial.println(horizontal.read());
}

void setstartpos(){
   horizontal.write(servohLimitLow);//(servoh);
   vertical.write(servovLimitLow);//(servov);
}

void setup() {
   if ( DEBUG ) {
       Serial.begin(9600);
   }
   power.hardwareDisable(PWR_I2C | PWR_SPI | PWR_USB | PWR_TIMER2);
   power.setSleepMode(POWERDOWN_SLEEP);
   horizontal.attach(9); //Horizontal(Up/Down) move motor pin
   vertical.attach(8); //Vertical(Left/Right) move motor pin
   setstartpos();
   delay(1000);
   if ( DEBUG ) {
      Serial.println("Starting from position:");
      Serial.println("---------------------------------------");
      servo_pos_print();
      Serial.println("---------------------------------------");
   }
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
         if (((avl - avr) > tol) && ((avl - avr) > 0)) {
            servov = servov - 1;
            if (servov < servovLimitLow) {
               servov = servovLimitLow;
            }
         } else
            if (((avr - avl) > tol) && ((avr - avl) > 0)) {
               servov = servov + 1;
               if (servov > servovLimitHigh) {
                  servov = servovLimitHigh;
               }
            } else
               if (avr == avl) {
                  delay (10);
               }
         if (servov != servov_old) {
            vertical.write(servov);
            delay (10);
            if ( DEBUG ) {
               servo_pos_print();
            }
         }
         servov_old = servov;
         if (((avt - avd) > tol) && ((avt - avd) > 0)) {
            servoh = servoh - 1;
            if (servoh < servohLimitLow) {
               servoh = servohLimitLow;
            }
            } else
               if (((avd - avt) > tol) && ((avd - avt) > 0)) {
                  servoh = servoh + 1;
                  if (servoh > servohLimitHigh) {
                     servoh = servohLimitHigh;
                  }
               } else
                  if (avt == avd) {
                     delay(10);
                  }
         if (servoh != servoh_old) {
            horizontal.write(servoh);
            delay (10);
            if ( DEBUG ) {
               servo_pos_print();
            }
         }
         servoh_old = servoh;
      delay(dtime);
   } else {
      if (parked == false) {
         tmr.stop();
         if ( DEBUG ) {
            Serial.println("Going to START Position. Prepare for Sunrise...");
         }
         setstartpos();
         delay(1000);
         parked = true;
         tmr.start();
      } else {
            tmr.stop();
            if ( DEBUG ) {
               Serial.println("It`s on start postion. Waiting for sunrise...");
               Serial.print("Going to PowerDown sleep for ");
               Serial.print(sleeptime_min);
               Serial.println(" minutes");
            }
            delay(100);
            power.sleepDelay(sleeptime_ms);
            if ( DEBUG ) {
               Serial.println("Waking up for check is Sunrise ?");
               servo_pos_print();
            }
            tmr.start();
      }
   }

   if (tmr.tick()) {
      if ( DEBUG ) {
         Serial.print(worktime_sec);
         Serial.println(" seconds of work");
         Serial.print("Going to PowerDown sleep for ");
         Serial.print(sleeptime_min);
         Serial.println(" minutes");
      }
      delay(100);
      power.sleepDelay(sleeptime_ms);
      if ( DEBUG ) {
            Serial.print("Waking up for ");
            Serial.print(worktime_sec);
            Serial.println(" seconds to work");
            servo_pos_print();
      }
      tmr.start();
   }
}
