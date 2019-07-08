#include <Servo.h>
#include <Arduino.h>

#define MODE1 1     // Gather samples and shoot without condithon  
#define MODE1 2     // Gather samples and shoot after sound detection
#define AmpMax (512)
#define MicSamples (1024 * 2)
#define VolumeGainFactorBits 3
#define SOUND_SPEED 343
#define MICRO_TO_MILI 1000000
#define FHT_N 256 // set to 256 point fht
#define MAX_X 120
#define MIN_X 60
#define MAX_Y 120
#define MIN_Y 85
#define SENSE_TIME 100 // Should be calculate precise
#define IN1 6
#define IN2 7
#define ENA 11
#define LOADER_SPEED 1000
#define LOADER_TIME 900
#define DELTA_ANGLE 5
#define RAISE_TRESH 10

int micAnalogData[4] = {0,0,0,0};
int prevMicData[4] = {0,0,0,0};
int micDCs[4] = {0,0,0,0};

Servo verticalMotor;
Servo horizontalMotor;

int verticalAngle;
int horizontalAngle;
int mode;

uint8_t micPins[4] = {A0, A1, A2, A3};

float micDistances[4] = {0, 0, 0, 0};
boolean isDetected[4] = {false,false,false,false};
int measureCount = 0;

unsigned long micDetectTime[4] = {0, 0, 0, 0};
unsigned long measureStartTime;

void calculateEnvDC()
{   
    for (int i = 0; i < 4; i++)
    {
        long prevAvg = -1, soundVolAvg = 0, soundSum = 0, sampleCount = 0;
        while (soundVolAvg != prevAvg || sampleCount < 1024)
        {
            prevAvg = soundVolAvg;
            int amp = analogRead(micPins[i]);
            // amp <<= VolumeGainFactorBits;
            soundSum += amp;
            sampleCount += 1;
            soundVolAvg = soundSum / sampleCount;
        }
        micDCs[i] = soundVolAvg;
    }
}

int measureVolume(int id)
{
    long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;
    for (int i = 0; i < MicSamples; i++)
    {
        int k = analogRead(micPins[id]);

        int amp = abs(k - micDCs[id]);
        amp <<= VolumeGainFactorBits;
        soundVolMax = max(soundVolMax, amp);
        soundVolAvg += amp;
        soundVolRMS += ((long)amp*amp);
    }
    soundVolAvg /= MicSamples;
    soundVolRMS /= MicSamples;
    float soundVolRMSflt = sqrt(soundVolRMS);
    float dB = 20.0 * log10(soundVolRMSflt/AmpMax);
 
    soundVolAvg = 100 * soundVolAvg / AmpMax; 
    soundVolMax = 100 * soundVolMax / AmpMax; 
    soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
    soundVolRMS = 10 * soundVolRMSflt / 7;
    return soundVolMax;
}

void readMicData()
{
    for (int i=0; i<4; i++) 
    {
        if (isDetected[i] == true)
            continue;
 
        micAnalogData[i] = measureVolume(i);
        isDetected[i] = (abs(prevMicData[i] - micAnalogData[i]) > RAISE_TRESH) ? true : false;
        Serial.print("isDetected[" + String(i) + "]: " + String(isDetected[i]) + "\t" + "DC: " + micDCs[i] + "\t" + "Analog: " + micAnalogData[i] + "\tprev: " + prevMicData[i] + "\t");
        Serial.println(abs(prevMicData[i] - micAnalogData[i]));
        // Serial.println("**********");
        if (isDetected[i])
            micDetectTime[i] = micros();

        prevMicData[i] = micAnalogData[i];
    }
    measureCount++; 

}

void finalizeMicDetectTime(unsigned long finalMicros)
{
    for (int i = 0; i < 4; i++)
    {
        // Serial.println("isDetected[" + String(i) + "]: " + String(isDetected[i]) + "\t" + "DC: " + micDCs[i] + "\t\t" + "Analog: " + micAnalogData[i] + "\t\t\tprev: " + prevMicData[i]);
        if (micDetectTime[i] == 0)
            micDetectTime[i] = finalMicros;
    }
    // Serial.println("**********");
}

boolean allDetected()
{
    return (isDetected[0] || isDetected[1] || isDetected[2] || isDetected[3]);
}

void resetAllDetections()
{
    for (int i = 0; i < 4; i++)
    {
        isDetected[i] = false;
        micDetectTime[i] = 0;
    }
}

void calculateAngles()
{
    for (int i = 0; i < 4; i++)
    {   
        // Serial.print("micDetectTime: ");
        // Serial.println(micDetectTime[i]);
        unsigned long diff = micDetectTime[i] - measureStartTime;
        // Serial.println(String(i) + ": " + diff); 
        micDistances[i] = diff * SOUND_SPEED / MICRO_TO_MILI;
        // Serial.print("micDistances: " + String(i));
        // Serial.println(micDistances[i]);
    }
    // Serial.println("^^^^^^^^^^^^");
    bool isPlusY = (micDistances[2] > micDistances[0]);
    bool isPlusX = (micDistances[3] > micDistances[1]);

    if (isPlusX)
    {
        Serial.print("x < 0");
        horizontalAngle += DELTA_ANGLE;
        if (horizontalAngle > 120)
            horizontalAngle = 120;
    }
    else if (micDistances[2] != micDistances[0])
    {
        Serial.print("x > 0");
        horizontalAngle -= DELTA_ANGLE;
        if (horizontalAngle < 60)
            horizontalAngle = 60;
    }

    // if (isPlusY)
    // {
    //     Serial.println(", y < 0");            
    //     verticalAngle += DELTA_ANGLE;
    //     if (verticalAngle > 100)
    //         verticalAngle = 100;
    // }
    // else if (micDistances[3] != micDistances[1])
    // {
    //     Serial.println(", y > 0") ;
    //     verticalAngle -= DELTA_ANGLE;
    //     if (verticalAngle < 80)
    //         verticalAngle = 80;
    // }

    Serial.print("Angles: ");
    Serial.print(horizontalAngle);
    Serial.print(", ");
    Serial.println(verticalAngle);
}

void actuateMotors()
{
    verticalMotor.write(verticalAngle);
    horizontalMotor.write(horizontalAngle);

    delay(100);
}

void shoot()
{
    digitalWrite(IN2, HIGH);
    delay(LOADER_TIME);
    digitalWrite(IN2, LOW);
}

void setup() 
{ 
    verticalAngle = 75;
    horizontalAngle = 90;
    Serial.begin(9600);
    verticalMotor.attach(9);
    horizontalMotor.attach(10);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, LOADER_SPEED);
    calculateEnvDC();
    mode = MODE1;
}

void loop() 
{   
    Serial.println("hellooo!");
    measureStartTime = micros();

    // resetAllDetections();
    while (micros() - measureStartTime < SENSE_TIME)
        readMicData();

    unsigned long finalMicros = micros();
    finalizeMicDetectTime(finalMicros);

    if (allDetected())
    {
        calculateAngles();
        actuateMotors();
        shoot();
        resetAllDetections();
    }
}