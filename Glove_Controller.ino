#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* Define RF pins */
#define CE_PIN   9
#define CSN_PIN 10

const byte slaveAddress[5] = {'R','x','A','A','A'};

RF24 radio(CE_PIN, CSN_PIN);                          // Create a Radio

float Vref[4];                                        // Array of voltage references to send to Echo

unsigned long currMillis;
unsigned long prevMillis;

unsigned long txIntervalMillis = 200;                 // Smaller interval = more frequent sending

void setup() 
{
    Serial.begin(9600);
    pinMode(A0,INPUT);                                // thumb sensor pin
    pinMode(A1,INPUT);                                // index finger sensor pin
    pinMode(A2,INPUT);                                // middle finger sensor pin

    Serial.println("Glove TX Starting");

    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.setRetries(3,5);                            // delay, count
    radio.openWritingPipe(slaveAddress);
}

/* Measures the voltage values of each sensor on glove controller
   and stores the values into Vref[] to serve as reference values
   for the robotic hand                                           */
void updateVref() 
{
    float val = (float) analogRead(A0);     
    Vref[0] = (float) (5 * (val/1024));
    float val2 = (float) analogRead(A1);
    Vref[1] = (float) (5 * (val2/1024));
    float val3 = (float) analogRead(A2);
    Vref[2] = (float) (5 * (val3/1024));

    Serial.print("Vref[0]: ");
    Serial.print(Vref[0]);
    Serial.print("Vref[1]: ");
    Serial.print(Vref[1]);
    Serial.print("Vref[2]: ");
    Serial.print(Vref[2]);
}

void send() 
{
    updateVref();                                     // Gets current voltage values
    bool rslt;
    rslt = radio.write( &Vref, sizeof(Vref) );        // Checks if Echo received Vref[]
    if (rslt) {
        Serial.println("  Acknowledge received");     
    }
    else {
        Serial.println("  Tx failed");
    }
}

void loop() 
{
    currMillis = millis();
    if (currMillis - prevMillis >= txIntervalMillis)  // Send data every interval amount of ms
    {
        send();
        prevMillis = millis();
    }
}
