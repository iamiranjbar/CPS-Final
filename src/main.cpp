#include <Servo.h>
#include <Arduino.h>

#define AmpMax (512)
#define MicSamples (1024 * 4)
#define VolumeGainFactorBits 1
#define SOUND_SPEED 343
#define MICRO_TO_MILI 1000
#define FHT_N 256 // set to 256 point fht
#define MAX_X 120
#define MIN_X 60
#define MAX_Y 120
#define MIN_Y 85
#define SENSE_TIME 1000 // Should be calculate precise

unsigned long micAnalogData[4] = {0,0,0,0};
int prevMicData[4] = {0,0,0,0};
int micDCs[4] = {0,0,0,0};

Servo verticalMotor;
Servo horizontalMotor;

int verticalAngle;
int horizontalAngle;

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
            amp <<= VolumeGainFactorBits;
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
        isDetected[i] = (abs(prevMicData[i] - micAnalogData[i]) > 0) ? true : false;
        if (isDetected[i])
            micDetectTime[i] = micros();

        prevMicData[i] = micAnalogData[i];
    }
    measureCount++; 

}

void finalizeMicDetectTime()
{
    for (int i = 0; i < 4; i++)
    {
        Serial.println("isDetected[" + String(i) + "]: " + String(isDetected[i]));
        if (micDetectTime[i] == 0)
            micDetectTime[i] = micros();
    }
    Serial.println("**********");
}

boolean allDetected()
{
    return (isDetected[0] && isDetected[1] && isDetected[2] && isDetected[3]);
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
        unsigned long diff = measureStartTime - micDetectTime[i];
        // Serial.println("Diff: " + String(diff));
        micDistances[i] = diff * SOUND_SPEED * MICRO_TO_MILI;
        // Serial.print("micDistances: " + String(i));
        // Serial.println(micDistances[i]);
    }

    bool isPlusY = (micDistances[2] > micDistances[0]);
    bool isPlusX = (micDistances[3] > micDistances[1]);

    if (isPlusX)
    {
        Serial.print("x < 0");
        horizontalAngle = 120;
    }
    else
    {
        Serial.print("x > 0");
        horizontalAngle = 60;
    }

    if (isPlusY)
    {
        Serial.println(", y < 0");            
        verticalAngle = 100;
    }
    else
    {
        Serial.println(", y > 0") ;
        verticalAngle = 80;
    }

    // Serial.print("Angles: ");
    // Serial.print(horizontalAngle);
    // Serial.print(", ");
    // Serial.println(verticalAngle);
}

void actuateMotors()
{
    verticalMotor.write(verticalAngle);
    horizontalMotor.write(horizontalAngle);

    delay(100);
}

void shoot()
{

}

void setup() 
{ 
    verticalAngle = 90;
    horizontalAngle = 90;
    Serial.begin(9600);
    verticalMotor.attach(9);
    horizontalMotor.attach(10);
    calculateEnvDC();
}

void loop() 
{   
    measureStartTime = micros();

    // resetAllDetections();
    while (micros() - measureStartTime < SENSE_TIME)
        readMicData();

    finalizeMicDetectTime();

    if (allDetected())
    {
        calculateAngles();
        actuateMotors();
        shoot();
        resetAllDetections();
    }
}