/*
 Example sketch for the Xbox Wireless Reciver library - developed by Kristian Lauszus
 It supports up to four controllers wirelessly
 For more information see the blog post: http://blog.tkjelectronics.dk/2012/12/xbox-360-receiver-added-to-the-usb-host-library/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <Servo.h>

USB Usb;
XBOXRECV Xbox(&Usb);

#define MFR1 26 //22
#define MFR2 27 //23
#define MFL1 28 //24
#define MFL2 29 //25

#define MBR1 22
#define MBR2 23
#define MBL1 24
#define MBL2 25

#define Flipper1 A4
#define Flipper2 A5
#define Flipper3 A6
#define Flipper4 A7

#define sensorR 30
#define sensorL 31

#define enL 11
#define enR 10

 int Xaxisp=9500;
 int Yaxisp=9500;
 int Xaxisn=-9500;
 int Yaxisn=-9500;
 int autogear=5;

//Servo servo1;
//Servo servo2;



void setup() {
  pinMode(MFR1, OUTPUT); //Orange
  pinMode(MFR2, OUTPUT); //Blue
  pinMode(MFL1, OUTPUT); //Red
  pinMode(MFL2, OUTPUT); //Black

  pinMode(MBR1, OUTPUT); //Orange
  pinMode(MBR2, OUTPUT); //Blue
  pinMode(MBL1, OUTPUT); //Red
  pinMode(MBL2, OUTPUT); //Black

  pinMode(Flipper1, OUTPUT); //Orange
  pinMode(Flipper2, OUTPUT); //Yellow
  pinMode(Flipper3, OUTPUT); //Orange
  pinMode(Flipper4, OUTPUT); //Yellow

  pinMode(sensorR, INPUT);
  pinMode(sensorL, INPUT);

  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  
  //servo1.attach(26);  //the pin for the servo control, and range if desired
 // servo1.write(1); //set initial servo position if desired
  //servo2.attach(27);  //the pin for the servo control, and range if desired
 // servo2.write(1); //set initial servo position if desired
  int n=0;
 
  
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

void linefollower(int leftS, int rightS)
{

  analogWrite(enL, 128); // Send PWM signal to L298N Enable pin
  analogWrite(enR, 128); // Send PWM signal to L298N Enable pin

  setalllow();
  // Black in the center white on both sides
  //both detect black line STOP
  if (leftS == HIGH && rightS == HIGH)
  {
  wrmotor(LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW);
  }

  //both dont detect black line FORWARD
  if (leftS == LOW && rightS == LOW)
  {
  wrmotor(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);       
  }

 //left detects black line => forward left slow
  if (leftS == HIGH && rightS == LOW)
  {
  wrmotor(HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW);
  }
  
 //right detects black line => forward right slow
  if (leftS == LOW && rightS == HIGH)
  {
  wrmotor(LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW);
  }

   analogWrite(enL, 0); // Send PWM signal to L298N Enable pin
  analogWrite(enR, 0); // Send PWM signal to L298N Enable pin
}


void setalllow()
{

digitalWrite(MFR1, LOW); // set all LOW
digitalWrite(MFR2, LOW);
digitalWrite(MFL1, LOW);
digitalWrite(MFL2, LOW);

digitalWrite(MBR1, LOW);
digitalWrite(MBR2, LOW);
digitalWrite(MBL1, LOW);
digitalWrite(MBL2, LOW);
          
/*
digitalWrite(Flipper1, LOW);   
digitalWrite(Flipper2, LOW);  
digitalWrite(Flipper3, LOW);   
digitalWrite(Flipper4, LOW); 
*/
}



void wrmotor(int MFL1_s, int MFL2_s, int MFR1_s, int MFR2_s, int MBL1_s, int MBL2_s, int MBR1_s, int MBR2_s)
{
digitalWrite(MFL1,MFL1_s );
digitalWrite(MFL2, MFL2_s );
digitalWrite(MFR1, MFR1_s);
digitalWrite(MFR2, MFR2_s);

digitalWrite(MBL1,MBL1_s );
digitalWrite(MBL2, MBL2_s );
digitalWrite(MBR1, MBR1_s);
digitalWrite(MBR2, MBR2_s);

setalllow();

}

void loop() {
 
  
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (Xbox.Xbox360Connected[i]) {
        if (Xbox.getButtonPress(L2, i) || Xbox.getButtonPress(R2, i)) {
          Serial.print("L2: ");
          Serial.print(Xbox.getButtonPress(L2, i));
          Serial.print("\tR2: ");
          Serial.println(Xbox.getButtonPress(R2, i));
          Xbox.setRumbleOn(Xbox.getButtonPress(L2, i), Xbox.getButtonPress(R2, i), i);

          autogear=Xbox.getButtonPress(R2, i)+1;
           Serial.print("\tAutogear: ");
           Serial.println(autogear);
        }

          setalllow(); // set all LOW
           

          int LXget=0,LYget=0;
          int RXget=0,RYget=0;

         

        if (Xbox.getButtonClick(START, i)) {
          Xbox.setLedMode(ALTERNATING, i);
          Serial.println(F("Start Autonomous"));


          Serial.println(F("Left sensor "));
          Serial.println(digitalRead(sensorL));
          Serial.println(F(" Right sensor"));
          Serial.println(digitalRead(sensorR));

          // line follower loop
          while (digitalRead(sensorL) == LOW || digitalRead(sensorR) == LOW)
          
          {        

          
          linefollower( digitalRead(sensorL), digitalRead(sensorR));

          delay(1);
           
          }
        
        
        
        }

        setalllow(); // set all LOW
         // Left-Analog joystick    
           if (Xbox.getAnalogHat(LeftHatX, i) > Xaxisp || Xbox.getAnalogHat(LeftHatX, i) < Xaxisn)
           {
            LXget=Xbox.getAnalogHat(LeftHatX, i);
            }
          
           if (Xbox.getAnalogHat(LeftHatY, i) > Yaxisp || Xbox.getAnalogHat(LeftHatY, i) < Yaxisn)
           {
            LYget=Xbox.getAnalogHat(LeftHatY, i);
           }
             
         // Right-Analog joystick    
        if (Xbox.getAnalogHat(RightHatX, i) > Xaxisp || Xbox.getAnalogHat(RightHatX, i) < Xaxisn)
           {
            RXget=Xbox.getAnalogHat(RightHatX, i);
            }
          
           if (Xbox.getAnalogHat(RightHatY, i) > Yaxisp || Xbox.getAnalogHat(RightHatY, i) < Yaxisn)
           {
            RYget=Xbox.getAnalogHat(RightHatY, i);
           }
        
        if (Xbox.getAnalogHat(RightHatX, i) > Xaxisp || Xbox.getAnalogHat(RightHatX, i) < Xaxisn || Xbox.getAnalogHat(LeftHatY, i) > Yaxisp || Xbox.getAnalogHat(LeftHatY, i) < Yaxisn || Xbox.getAnalogHat(RightHatX, i) > Xaxisp || Xbox.getAnalogHat(RightHatX, i) < Xaxisn || Xbox.getAnalogHat(RightHatY, i) > Yaxisp || Xbox.getAnalogHat(RightHatY, i) < Yaxisn) 
        {
         ////////// LEFT ANALOG //////////

        analogWrite(enL, 255); // Send PWM signal to L298N Enable pin
        analogWrite(enR, 255);// Send PWM signal to L298N Enable pin
         
          // FORWARD RIGHT FAST
          if (LXget > Xaxisp && LYget ==0) {
          wrmotor(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH);
          }

           // FORWARD RIGHT SLOW 
          if (LXget > Xaxisp && LYget > Yaxisp ) {
          wrmotor(LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW);
          }
          
        // FORWARD LEFT SLOW
       if (LXget < Xaxisn && LYget > Yaxisp) {
         wrmotor(HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW);
          }

          // FORWARD LEFT FAST
           if (LXget < Xaxisn && LYget==0 ) {
           wrmotor(HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
          }

          // FORWARD 
          if (LYget > Yaxisp && LXget==0 ) {
            wrmotor(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);       
          }
          
          // BACK 
           if (LYget < Yaxisn  && LXget==0) {
            wrmotor(HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH);       
          }

           // BACK RIGHT SLOW 
          if (LXget > Xaxisp && LYget < Yaxisn ) {
          wrmotor(LOW, LOW, LOW, HIGH, LOW, LOW, LOW, HIGH);
 
          }
          
        // BACK LEFT SLOW
       if (LXget < Xaxisn && LYget < Yaxisn) {
          wrmotor(LOW, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW);
           }
  
          setalllow(); // set all LOW
         

          ////////// LEFT ANALOG //////////
          
        }

         ////////// OTHER BUTTONS //////////


        if (Xbox.getButtonClick(UP, i)) {
          Xbox.setLedOn(LED1, i);
          Serial.println(F("Up"));
        }
        if (Xbox.getButtonClick(DOWN, i)) {
          Xbox.setLedOn(LED4, i);
          Serial.println(F("Down"));
        }
        if (Xbox.getButtonClick(LEFT, i)) {
          Xbox.setLedOn(LED3, i);
          Serial.println(F("Left"));
        }
        if (Xbox.getButtonClick(RIGHT, i)) {
          Xbox.setLedOn(LED2, i);
          Serial.println(F("Right"));
        }

        if (Xbox.getButtonClick(START, i)) {
          Xbox.setLedMode(ALTERNATING, i);
          Serial.println(F("Start"));
        }
        if (Xbox.getButtonClick(BACK, i)) {
          Xbox.setLedBlink(ALL, i);
          Serial.println(F("Back"));
        }
        if (Xbox.getButtonClick(L3, i))
          Serial.println(F("L3"));
        if (Xbox.getButtonClick(R3, i))
          { 
            Serial.println(F("R3"));
          }

        if (Xbox.getButtonClick(L1, i))
          {
            Serial.println(F("L1"));
           
          }
        if (Xbox.getButtonClick(R1, i))
          {
            Serial.println(F("R1"));          
            
          }
        if (Xbox.getButtonClick(XBOX, i)) {
          Xbox.setLedMode(ROTATING, i);
          Serial.print(F("Xbox (Battery: "));
          Serial.print(Xbox.getBatteryLevel(i)); // The battery level in the range 0-3
          Serial.println(F(")"));
        }
        if (Xbox.getButtonClick(SYNC, i)) {
          Serial.println(F("Sync"));
          Xbox.disconnect(i);
        }

        if (Xbox.getButtonClick(A, i))
          {
            Serial.println(F("A"));
            
           }
        if (Xbox.getButtonClick(B, i))
          {
            Serial.println(F("B"));
          

          }
        if (Xbox.getButtonClick(X, i))
          {
            Serial.println(F("X"));
          
          
          
          }
          
        if (Xbox.getButtonClick(Y, i))
          {
            Serial.println(F("Y"));
                      
          
          }

        ////////// OTHER BUTTONS //////////

      }
    }
  }
}
