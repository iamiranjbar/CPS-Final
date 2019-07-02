#include <Servo.h>
#include <Arduino.h>

#define AmpMax (512)
#define MicSamples (1024*2)
#define VolumeGainFactorBits 0
#define MAX_X 120
#define MIN_X 60
#define MAX_Y 120
#define MIN_Y 85
#define SENSE_TIME 100000 // Should be calculate precise

Servo verticalMotor;
Servo horizontalMotor;
int verticalAngle;
int horizontalAngle;
int prevMicData[4] = {0,0,0,0};
int micAnalogData[4] = {0,0,0,0};
int micDCs[4] = {0,0,0,0};
long micDetectTime[4] = {-1,-1,-1,-1};
uint8_t micPins[4] = {A0, A1, A2, A3};
int horizontalMicDiffs = 0;
int verticalMicDiffs = 0;
boolean isDetected[4] = {false,false,false,false};
int measureCount = 0;

int calculateEnvDC()
{   
    for (int i = 0; i < 4; i++)
    {
        long prevAvg = -1, soundVolAvg = 0, soundSum = 0, sampleCount = 0;
        while (soundVolAvg != prevAvg || sampleCount < 1024)
        {
            prevAvg = soundVolAvg;
            int amp = analogRead(pin);
            amp <<= VolumeGainFactorBits;
            soundSum += amp;
            sampleCount += 1;
            soundVolAvg = soundSum / sampleCountl
        }
        micDCs[i] = soundVolAvg;
    }
}

int MeasureVolume(int id)
{
    long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
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
    float dB = 20.0*log10(soundVolRMSflt/AmpMax);
 
    soundVolAvg = 100 * soundVolAvg / AmpMax; 
    soundVolMax = 100 * soundVolMax / AmpMax; 
    soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
    soundVolRMS = 10 * soundVolRMSflt / 7;
 
    Serial.println("********************************************");
    Serial.print("Time: " + String(millis() - t0));
    Serial.print(" Amp: Max: " + String(soundVolMax));
    Serial.print(" % Avg: " + String(soundVolAvg));
    Serial.print(" % RMS: " + String(soundVolRMS));
    Serial.println(" % dB: " + String(dB,3));
    Serial.println("********************************************");
    return soundVolMax;
}

void readMicData()
{
    for (int i=0; i<4; i++) 
    {
        if (isDetected[i] == true)
        {
            continue;
        }
        micAnalogData[i] = MeasureVolume(i);
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
        if (micDetectTime[i] == -1)
            micDetectTime[i] = micros();
    }
}

boolean allDetected()
{
    return (isDetected[0] && isDetected[1] && isDetected[2] && isDetected[3]);
}

boolean reachedTreshold()
{
    return (horizontalMicDiffs > 2 || verticalMicDiffs > 2 || horizontalMicDiffs < -2 || verticalMicDiffs < -2);
}

void resetAllDetections()
{
    for (int i = 0; i < 4; i++)
    {
        isDetected[i] = false;
        micDetectTime[i] = -1;
    }
}

void calculateAngles()
{
    Serial.print(constrain(horizontalMicDiffs, -200, 200));
    Serial.print(":");
    Serial.print(constrain(verticalMicDiffs, -200, 200));
    Serial.println("&");
    delay(1000);
    // Serial.println(millis());
    // previousParameters[0] += constrain(horizontalMicDiffs, -200, 200) * 4;
    // previousParameters[1] += constrain(verticalMicDiffs, -200, 200) * 4;
    // previousParameters[0] = (previousParameters[0] < MIN_X) ? MIN_X : ((previousParameters[0] > MAX_X) ? MAX_X : previousParameters[0]);
    // previousParameters[1] = (previousParameters[1] < MIN_Y) ? MIN_Y : ((previousParameters[1] > MAX_Y) ? MAX_Y : previousParameters[1]);
}

void actuateMotors()
{
    
}

void shoot()
{

}

void resetDiffs()
{
    horizontalMicDiffs = 0;
    verticalMicDiffs = 0;
}

void setup() 
{ 
    verticalAngle = 140;
    horizontalAngle = 20;
    Serial.begin(9600);
    verticalMotor.attach(9);
    horizontalMotor.attach(10);
    calculateEnvDC();
}

void loop() 
{   
    int measureStartTime = micros();
    while (micros() - measureStartTime < SENSE_TIME)
    {
        readMicData();
    }
    finalizeMicDetectTime();
    if (allDetected() || reachedTreshold())
    {
        resetAllDetections();
        calculateAngles();
        actuateMotors();
        shoot();
        resetDiffs();
    }
}