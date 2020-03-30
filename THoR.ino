#include <XBOXRECV.h>
#include <SPI.h>
#include <Servo.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

// -------------------
// DEFINE PINS HERE
// -------------------

// ------ FRONT ---------
#define MFR1 26 //22
#define MFR2 27 //23
#define MFL1 28 //24
#define MFL2 29 //25

// ------ BACK ---------
#define MBR1 22
#define MBR2 23
#define MBL1 24
#define MBL2 25

// ------ SENSORS --------
#define sensorR 30
#define sensorL 31

// ------ PWM(?) --------
#define enL 11
#define enR 10

// -------------------
// MAIN CLASS
// -------------------

// Need to declare these outside of the class due
// to unknown compilation error. 
USB THoR; // USB Object Definition
XBOXRECV THoRRCV(&THoR); // Xbox Receiver Object

class THoRController
{
    // ----- INSERT CONTROL FUNCTIONS BELOW PUBLIC LINE -----
    public:
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
    }

    void wrmotor(int MFL1_s, int MFL2_s, int MFR1_s, int MFR2_s, int MBL1_s, int MBL2_s, int MBR1_s, int MBR2_s)
    {
        digitalWrite(MFL1, MFL1_s);
        digitalWrite(MFL2, MFL2_s);
        digitalWrite(MFR1, MFR1_s);
        digitalWrite(MFR2, MFR2_s);

        digitalWrite(MBL1, MBL1_s);
        digitalWrite(MBL2, MBL2_s);
        digitalWrite(MBR1, MBR1_s);
        digitalWrite(MBR2, MBR2_s);

        setalllow();
    }

    void engageMotors()
    {
        if (
            THoRRCV.getAnalogHat(LeftHatY, 0) > axesSense[0] ||
            THoRRCV.getAnalogHat(LeftHatY, 0) < axesSense[0] ||
            THoRRCV.getAnalogHat(LeftHatX, 0) > axesSense[1] ||
            THoRRCV.getAnalogHat(LeftHatX, 0) < axesSense[1]
           ) 
        {
            // -----------------------------
            // ------- LEFT JOYSTICK -------
            // -----------------------------
            analogWrite(enL, 255); // Send PWM signal to L298N Enable pin
            analogWrite(enR, 255);// Send PWM signal to L298N Enable pin
            
            // FORWARD RIGHT FAST
            if (axes[1] > axesSense[1] && axes[0] == 0) 
            {
                wrmotor(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH);
            }

            // FORWARD RIGHT SLOW 
            if (axes[1] > axesSense[1] && axes[0] > axesSense[0]) 
            {
                wrmotor(LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW);
            }
            
            // FORWARD LEFT SLOW
            if (axes[1] < axesSense[1] && axes[0] > axesSense[0]) 
            {
                wrmotor(HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW);
            }

            // FORWARD LEFT FAST
            if (axes[1] < axesSense[1] && axes[0] == 0) 
            {
            wrmotor(HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
            }

            // FORWARD 
            if (axes[0] > axesSense[0] && axes[1] == 0) 
            {
                wrmotor(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);       
            }
            
            // BACK 
            if (axes[0] < axesSense[0] && axes[1] == 0) 
            {
                wrmotor(HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH);       
            }

            // BACK RIGHT SLOW 
            if (axes[1] > axesSense[1] && axes[0] < axesSense[0]) 
            {
                wrmotor(LOW, LOW, LOW, HIGH, LOW, LOW, LOW, HIGH);
            }

            // BACK LEFT SLOW
            if (axes[1] < axesSense[1] && axes[0] < axesSense[0]) 
            {
                wrmotor(LOW, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW);
            }
            setalllow(); // set all LOW
        }

        // -----------------------------
        // ------ RIGHT JOYSTICK -------
        // -----------------------------
        if (
            THoRRCV.getAnalogHat(RightHatX, 0) > axesSense[1] ||
            THoRRCV.getAnalogHat(RightHatX, 0) < axesSense[1] ||
            THoRRCV.getAnalogHat(RightHatY, 0) > axesSense[0] ||
            THoRRCV.getAnalogHat(RightHatY, 0) < axesSense[0]
           ) 
        {
            analogWrite(enL, 255); // Send PWM signal to L298N Enable pin
            analogWrite(enR, 255);// Send PWM signal to L298N Enable pin
            
            // FORWARD RIGHT FAST
            if (axes[3] > axesSense[1] && axes[2] == 0) 
            {
                wrmotor(LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH);
            }

            // FORWARD RIGHT SLOW 
            if (axes[3] > axesSense[1] && axes[2] > axesSense[0]) 
            {
                wrmotor(LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW);
            }
            
            // FORWARD LEFT SLOW
            if (axes[3] < axesSense[1] && axes[2] > axesSense[0]) 
            {
                wrmotor(HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW);
            }

            // FORWARD LEFT FAST
            if (axes[3] < axesSense[1] && axes[2] == 0 ) 
            {
                wrmotor(HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW);
            }

            // FORWARD 
            if (axes[2] > axesSense[0] && axes[3] == 0 ) 
            {
                wrmotor(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);       
            }
            
            // BACK 
            if (axes[2] < axesSense[0]  && axes[3] == 0) 
            {
                wrmotor(HIGH, LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH);       
            }

            // BACK RIGHT SLOW 
            if (axes[3] > axesSense[1] && axes[2] < axesSense[0]) 
            {
                wrmotor(LOW, LOW, LOW, HIGH, LOW, LOW, LOW, HIGH);
            }
            
            // BACK LEFT SLOW
            if (axes[3] < axesSense[1] && axes[2] < axesSense[0]) 
            {
                wrmotor(LOW, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW);
            }
            setalllow(); // set all LOW
        }
    }

    void updateAxes()
    {
        if (THoRRCV.XboxReceiverConnected && THoRRCV.Xbox360Connected[0]) 
        {
            // ----- LEFT JOYSTICK -----
            if (THoRRCV.getAnalogHat(LeftHatY, 0) > axesSense[0] || 
            THoRRCV.getAnalogHat(LeftHatY, 0) < -axesSense[0]) 
            {
                axes[0] = THoRRCV.getAnalogHat(LeftHatY, 0);
            }

            if (THoRRCV.getAnalogHat(LeftHatX, 0) > axesSense[1] || 
            THoRRCV.getAnalogHat(LeftHatX, 0) < -axesSense[1]) 
            {
                axes[1] = THoRRCV.getAnalogHat(LeftHatX, 0);
            }

            // ----- RIGHT JOYSTICK -----
            if (THoRRCV.getAnalogHat(RightHatY, 0) > axesSense[2] || 
            THoRRCV.getAnalogHat(RightHatY, 0) < -axesSense[2]) 
            {
                axes[2] = THoRRCV.getAnalogHat(RightHatY, 0);
            }

            if (THoRRCV.getAnalogHat(RightHatX, 0) > axesSense[3] || 
            THoRRCV.getAnalogHat(RightHatX, 0) < -axesSense[3]) 
            {
                axes[3] = THoRRCV.getAnalogHat(RightHatX, 0);
            }
        }
    }

    void startAutonomous()
    {
        THoRRCV.setLedMode(ALTERNATING, 0);
        Serial.println(F("Start Autonomous"));

        // Line Follower loop
        while (digitalRead(sensorL) == LOW || digitalRead(sensorR) == LOW)
        {        
            linefollower(digitalRead(sensorL), digitalRead(sensorR));
            Serial.println(F("Left sensor "));
            Serial.println(digitalRead(sensorL));
            Serial.println(F(" Right sensor"));
            Serial.println(digitalRead(sensorR));
            delay(1);
        }
        setalllow();
    }

    void actionListener()
    {
        THoR.Task();
        setalllow();

        // ----- AUTONOMOUS LISTENER -----
        if (THoRRCV.getButtonClick(START, 0)) 
        {
            startAutonomous();
        }

        // ----- AXES LISTENER -----
        updateAxes();

        // ----- MOTOR LISTENER -----
        engageMotors();
    }

    // ----- ANY DECLARED VARIABLES BELOW PRIVATE LINE -----
    private:
    int axes[4];
    int axesSense[4] = {9500, 9500, 9500, 9500};

    /*
    Axes Array Structure:
    LY        LX        RY        RX
    ^ axes[0] ^ axes[1] ^ axes[2] ^ axes[3] 
    (same applies for axesSensitivity)
    */
} THoRC;

void setup()
{

    // -------------------
    // USB INIT
    // -------------------
    Serial.begin(115200);
    if (THoR.Init() == -1) 
    {
        Serial.print(F("\r\nFailure to connect to USB Shield."));
        while (1);
    }
    Serial.print(F("\r\nInitialization complete."));

    // -------------------
    // MOTOR PIN MODES
    // -------------------
    pinMode(MFR1, OUTPUT); //Orange
    pinMode(MFR2, OUTPUT); //Blue
    pinMode(MFL1, OUTPUT); //Red
    pinMode(MFL2, OUTPUT); //Black

    pinMode(MBR1, OUTPUT); //Orange
    pinMode(MBR2, OUTPUT); //Blue
    pinMode(MBL1, OUTPUT); //Red
    pinMode(MBL2, OUTPUT); //Black

/*
    pinMode(Flipper1, OUTPUT); //Orange
    pinMode(Flipper2, OUTPUT); //Yellow
    pinMode(Flipper3, OUTPUT); //Orange
    pinMode(Flipper4, OUTPUT); //Yellow
*/

    pinMode(sensorR, INPUT);
    pinMode(sensorL, INPUT);

    pinMode(enL, OUTPUT);
    pinMode(enR, OUTPUT);
}

void loop()
{
    THoRC.actionListener();
    THoRC.engageMotors();
}
