#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* Define RF pins */
#define CE_PIN   9
#define CSN_PIN 10

/* Finger motor pin format
   - enable pin for finger motor
   - pin for positive terminal of finger motor
   - pin for negative terminal of finger motor
*/

// Thumb
int TenA = 3;
int T1 = 4;
int T2 = 2;

// Index
int IenA = 6; 
int I1 = 8;
int I2 = 7;

// Middle
// Pins for middle finger are analog pins configured as outputs
// when used, they are called directly using pins A4 and A5
int MenA = 5;

// Array of voltage values read off of sensors on robotic hand
float control[4];

// Calibration values
// Change these according to max flex voltage off of robo hand
float thumbUF = 2.75;
float indexUF = 1.75;
float middleUF = 1.85;

// Values used to make sure robo fingers are moving
float prevThumb;
float prevIndex;
float prevMiddle;

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};


RF24 radio(CE_PIN, CSN_PIN);

// Array holds values received from glove controller
float Vref[4];
bool newData = false;

void getData() {
    if ( radio.available() ) {
        radio.read( &Vref, sizeof(Vref) );
        newData = true;
        Serial.println("NEW DATA");
    }
}

// Updates voltage measurements off robo fingers
void updateVm() {
    float val = (float) analogRead(A0);     // update thumb
    control[0] = (float) (5 * (val/1024));
    float val2 = (float) analogRead(A1);    // update index
    control[1] = (float) (5 * (val2/1024));
    float val3 = (float) analogRead(A2);    // update middle
    control[2] = (float) (5 * (val3/1024));
}

// Used to check if glove finger is moving by seeing if the difference measured is changing
// if difference is not changing, motor should stop moving 
// y is the newly read bend at robo finger, x is last read value of robo finger sensor
// returns true if y != x, false if y == x
bool changing(float x, float y)
{
  if (x != y)
  {
    return true;
  }
  return false;
}

// Used for debugging
void showData() {
    if (newData == true) {
        Serial.println("---THUMB---");
        Serial.print("GloveThumb =  ");
        Serial.println(Vref[0]);
        Serial.print("RoboThumb: ");
        Serial.println(control[0]);
        Serial.print("GloveThumb - RoboThumb = ");
        Serial.println(Vref[0]-control[0]);

        Serial.println("---INDEX---");
        Serial.print("GloveIndex =  ");
        Serial.println(Vref[1]);
        Serial.print("RoboIndex: ");
        Serial.println(control[1]);
        Serial.print("GloveIndex - RoboIndex = ");
        Serial.println(Vref[1]-control[1]);

        Serial.println("---MIDDLE---");
        Serial.print("GloveMiddle =  ");
        Serial.println(Vref[2]);
        Serial.print("RoboMiddle: ");
        Serial.println(control[2]);
        Serial.print("GloveMiddle - RoboMiddle = ");
        Serial.println(Vref[2]-control[2]);

        Serial.println("---MOVEMENT---");
    }
    
}

// Initiate input and output pins and move robo fingers to full unflexed position
void setup() {
    Serial.begin(9600);
    pinMode(A0,INPUT);
    pinMode(A1,INPUT);
    pinMode(A2,INPUT);
    pinMode(TenA, OUTPUT);
    pinMode(T1, OUTPUT);
    pinMode(T2, OUTPUT);
    pinMode(IenA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(MenA, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A5, OUTPUT);

    // reset middle
    while(1)
    {
      updateVm();
      if(control[2] > middleUF)
      {
        digitalWrite(A4, LOW);
      digitalWrite(A5,HIGH);
      analogWrite(MenA,50);
      Serial.println("middle");
      Serial.println(control[2]);
      }
      if(control[2] < middleUF)
      {
        break;
      }
    }
    analogWrite(MenA,0);

    // reset index
    while(1)
    {
      updateVm();
      if(control[1] > indexUF)
      {
        digitalWrite(I1, LOW);
      digitalWrite(I2,HIGH);
      analogWrite(IenA,50);
      Serial.println("index");
      Serial.println(control[1]);
      }
      if(control[1] < indexUF)
      {
        break;
      }
    }
    analogWrite(IenA,0);

    // reset thumb
    while(1)
    {
      updateVm();
      if(control[0] > thumbUF)
      {
        digitalWrite(T1, LOW);
      digitalWrite(T2,HIGH);
      analogWrite(TenA,50);
      Serial.println("thumb");
      Serial.println(control[0]);
      }
      if(control[0] < thumbUF)
      {
        break;
      }
    }
    analogWrite(TenA,0);
   
    Serial.println("Robot Receive Starting");
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();
}

void loop() {
    getData();
    showData();
    updateVm();
    if (newData == true)
    {
      // Thumb
      // STOP 
      if (abs(control[0] - Vref[0]) <= 0.1 || Vref[0] > 3.45 || control[0] < 2.65)                            // If the difference between ref voltage and measured voltage is within tolerance, stop motor
      {
        analogWrite(TenA,0);
        Serial.println("thumb stop");
      }
      // MOVE
      else if ((abs(control[0] - Vref[0]) > 0.1) && changing(control[0],prevThumb) == true)  // If the difference in Reference voltage and measured voltage is too large, move the motor
        {
            if (control[0] > Vref[0])               // When the Measured voltage is less than the reference voltage, direction is clockwise
            {
              digitalWrite(T1, LOW);
              digitalWrite(T2,HIGH);
              Serial.println("thumb unflex");  // Unflex
              analogWrite(TenA,100);             // move motor at constant speed
            }
            else //if (control <= Vref[0])     // When Measured voltage is greater than  the reference voltage, direction is counter clockwise
            {
              digitalWrite(T2, LOW);
              digitalWrite(T1,HIGH);
              Serial.println("thumb flex");  // Flex
              analogWrite(TenA,100);             // move motor at constant speed
            }
        }
      
    

     // Index
      if (abs(control[1] - Vref[1]) <= 0.1 || Vref[1] > 2.75 || Vref[1] < 1.7)                            // If the difference between ref voltage and measured voltage is within tolerance, stop motor
      {
        analogWrite(IenA,0);
        Serial.println("index stop");
      }
      else if ((abs(control[1] - Vref[1]) > 0.1) && changing(control[1],prevIndex) == true)  // If the difference in Reference voltage and measured voltage is too large, move the motor
      {
          if (control[1] > Vref[1])               // When the Measured voltage is less than the reference voltage, direction is clockwise
          {
            digitalWrite(I1, LOW);
            digitalWrite(I2,HIGH);
            Serial.println("index unflex");  // Unflex
            analogWrite(IenA,100);             // move motor at constant speed
          }
          else     // When Measured voltage is greater than  the reference voltage, direction is counter clockwise
          {
            digitalWrite(I2, LOW);
            digitalWrite(I1,HIGH);
            Serial.println("index flex");  // Flex
            analogWrite(IenA,100);             // move motor at constant speed
          }
          
      }
      

      // Middle
      if (abs(control[2] - Vref[2]) <= 0.1 || control[2] > 3.40 || Vref[2] < 1.75)                            // If the difference between ref voltage and measured voltage is within tolerance, stop motor
      {
        analogWrite(MenA,0);
        Serial.println("middle stop");
      }
      else if ((abs(control[2] - Vref[2]) > 0.1) && changing(control[2],prevMiddle) == true)  // If the difference in Reference voltage and measured voltage is too large, move the motor
      {
          if (control[2] > Vref[2])               // When the Measured voltage is less than the reference voltage, direction is clockwise
          {
            digitalWrite(A4, LOW);
            digitalWrite(A5,HIGH);
            Serial.println("middle unflex");   // Unflex
            analogWrite(MenA,100);             // move motor at constant speed
          }
          else     // When Measured voltage is greater than  the reference voltage, direction is counter clockwise
          {
            digitalWrite(A5, LOW);
            digitalWrite(A4,HIGH);
            Serial.println("middle flex");   // Flex
            analogWrite(MenA,100);             // move motor at constant speed
          }
          
      }

      newData = false;
      prevThumb = control[0];
      prevIndex = control[1];
      prevMiddle = control[2];
      
    }
}
