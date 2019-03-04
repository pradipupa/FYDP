#include <Stepper.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "arduinoFFT.h"
#include "math.h"
#include "Queue.h"


#define SAMPLING_FREQUENCY 180 //Hz, must be less than 10000 due to ADC

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

int16_t ax, ay, az;

unsigned int sampling_period_us;
unsigned long microseconds;

long timer = 0;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

const uint8_t motorPin1  = 5;  // Pin 14 of L293
const uint8_t motorPin2  = 6;  // Pin 10 of L293
const uint8_t motorPin3  = 10; // Pin  7 of L293

void setup() {

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
 
    //pinMode(LED_PIN, OUTPUT);
}

void loop() {
    const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
    uint16_t iterations = 10;
    uint16_t j = 0, i = 0;
    int count = 0;

    //iterations = iterations*0.65;
    Queue <double> domFreqX;
    double domFreq;
//    double domFreqY;
//    double domFreqZ;

//    double domFreqY[iterations];
//    double domFreqZ[iterations];

    double vRealX[samples];
//    double vRealY[samples];
//    double vRealZ[samples];

    double vImagX[samples];
//    double vImagY[samples];
    //double vImagZ[samples];

    uint8_t withinTremorRange = 0;
    uint8_t outsideTremorRange = 0;
    double average = 0;
    double stdDev = 0;
    double temp = 0;
    double confidenceLevel = 0;

    count = count++;
    if (count > 10) {
      domFreqX.dequeue();
      count--;
    }
    for(j = 0; j < samples; j++) {
        microseconds = micros();
//      accelgyro.getAcceleration(&ax, &ay, &az);
//      Serial.println(analogRead(0));   
//      vRealX[j] = (double)ax;

        vRealX[j] = analogRead(0);
//        vRealY[j] = analogRead(1);
        //vRealZ[j] = analogRead(2);

        vImagX[j] = 0;
//        vImagY[j] = 0;
        //vImagZ[j] = 0;

      while(micros() < (microseconds + sampling_period_us)){}
    }

    //------------------------------------ FFT computation on x-axis sensor readings ------------------------------------
    FFT.Windowing(vRealX, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
    FFT.Compute(vRealX, vImagX, samples, FFT_FORWARD); // Compute FFT 
    FFT.ComplexToMagnitude(vRealX, vImagX, samples); // Compute magnitudes 
    SquareArray(vRealX, samples);
    domFreq = FFT.MajorPeak(vRealX, samples, SAMPLING_FREQUENCY);
    Serial.println(domFreq);
    domFreqX.enqueue(domFreq);
    
    
    // Resetting the imaginary axis array for reuse
//    for(j = 0; j < samples; j++) {
//        vImagX[j] = 0;
//    }

//    //------------------------------------ FFT computation on y-axis sensor readings ------------------------------------
//    FFT.Windowing(vRealY, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
//    FFT.Compute(vRealY, vImagY, samples, FFT_FORWARD); // Compute FFT 
//    FFT.ComplexToMagnitude(vRealY, vImagY, samples); // Compute magnitudes 
//    SquareArray(vRealY, samples);
//    domFreqY = FFT.MajorPeak(vRealY, samples, SAMPLING_FREQUENCY);
//    
//    Serial.println(domFreqY);
//
//    // Resetting the imaginary axis array for reuse
//    for(j = 0; j < samples; j++) {
//        vImagX[j] = 0;
//    }


    //------------------------------------ FFT computation on z-axis sensor readings ------------------------------------
//    FFT.Windowing(vRealZ, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
//    FFT.Compute(vRealZ, vImagX, samples, FFT_FORWARD); // Compute FFT 
//    FFT.ComplexToMagnitude(vRealZ, vImagX, samples); // Compute magnitudes
//    SquareArray(vRealZ, samples);
//    domFreqZ = FFT.MajorPeak(vRealZ, samples, SAMPLING_FREQUENCY);
//    
//    Serial.println(domFreqZ);

    //------------------------------------ End of FFT Computation ------------------------------------
    
////
//    for(j = 0; j < count; j++) {
//        if ((domFreqX.Node.item > 10.00) || (domFreqX.Node.item < 3.00)) {
//            outsideTremorRange++;
//        } else {
//          withinTremorRange++;
//          average += domFreqX.Node.item;
//          Serial.print("Average = ");
//          Serial.print("Average = ");
//        }
//        domFreqX = domFreqX.next;
//    }
    
//
//    //Serial.println("domFreqX[withinTremorRange]");
//    /*for(j = 0; j < withinTremorRange; j++) {
//        Serial.print(domFreqX[j]);
//        Serial.print("\n");
//    }*/
//
//    i = 0;
//    if (withinTremorRange/iterations > 0.65) {
//        average = average/withinTremorRange;
//        for (j=0; j<withinTremorRange; j++) {
//           temp += sq(domFreqX[j] - average); // variance
//        }
//        stdDev = sqrt(temp/withinTremorRange); // standard deviation 
//
//        Serial.println("X axis: Tremor detected!");
//        confidenceLevel = (1 - (abs(stdDev-3.5)/3.5))*100;
//        Serial.print("Confidence Level: ");
//        Serial.println(confidenceLevel);
//
//        if (confidenceLevel > 60.00) {
//          digitalWrite(motorPin1, HIGH);
//          digitalWrite(motorPin2, HIGH);
//          digitalWrite(motorPin3, HIGH);
//        }
//    } else {
//      Serial.println("X axis: No tremor detected!"); 
//      i++;
//    }
//
//    withinTremorRange = 0;
//    outsideTremorRange = 0;
//    for(j=0; j<samples; j++) {
//        if ((domFreqY[j] > 10.00) || (domFreqY[j] < 3.00)) {
//            outsideTremorRange++;
//        } else {
//          domFreqY[withinTremorRange] = domFreqY[j];
//          withinTremorRange++;
//          average += domFreqY[j];
//        }
//    }
//
//    
//    /*Serial.println("domFreqY[withinTremorRange]");
//    for(j = 0; j < withinTremorRange; j++) {
//        Serial.print(domFreqY[j]);
//        Serial.print("\n");
//    }*/
//
//    if (withinTremorRange/iterations > 0.65) {      
//        average = average/withinTremorRange;
//        for (j=0; j<withinTremorRange; j++) {
//           temp += sq(domFreqY[j] - average); // variance
//        }
//        stdDev = sqrt(temp/withinTremorRange); // standard deviation 
//
//        Serial.println("Y axis: Tremor detected!");
//        confidenceLevel = (1 - (abs(stdDev-3.5)/3.5))*100;
//
//        Serial.print("Confidence Level: ");
//        Serial.println(confidenceLevel);
//
//        if (confidenceLevel > 60.00) {
//          digitalWrite(motorPin1, HIGH);
//          digitalWrite(motorPin2, HIGH);
//          digitalWrite(motorPin3, HIGH);
//        }
//    } else {
//      Serial.println("Y axis: No tremor detected!"); 
//      i++;
//    }
//
//    withinTremorRange = 0;
//    outsideTremorRange = 0;
//    for(j=0; j<samples; j++) {
//        if ((domFreqZ[j] > 10.00) || (domFreqZ[j] < 3.00)) {
//            outsideTremorRange++;
//        } else {
//          domFreqZ[withinTremorRange] = domFreqZ[j];
//          withinTremorRange++;
//          average += domFreqZ[j];
//        }
//    }
//    /*Serial.println("domFreqZ[withinTremorRange]");
//    for(j = 0; j < withinTremorRange; j++) {
//        Serial.print(domFreqZ[j]);
//        Serial.print("\n");
//    }*/
//    if (withinTremorRange/iterations > 60) {
//        average = average/withinTremorRange;
//        Serial.print("Z average = "); Serial.print("\t");
//        Serial.print(average);
//        Serial.print("\n");        
//
//        for (j=0; j<withinTremorRange; j++) {
//           temp += sq(domFreqZ[j] - average); // variance
//        }
//        stdDev = sqrt(temp/withinTremorRange); // standard deviation 
//        //Serial.print("Z stdDev = "); Serial.print("\t");
//        //Serial.print(stdDev);
//        //Serial.print("\n");        
//
//        Serial.println("Z axis: Tremor detected!");
//        confidenceLevel = (1 - (abs(stdDev-3.5)/3.5))*100;
//        Serial.print("Confidence Level: ");
//        Serial.println(confidenceLevel);
//        if (confidenceLevel > 60.00) {
//          digitalWrite(motorPin1, HIGH);
//          digitalWrite(motorPin2, HIGH);
//          digitalWrite(motorPin3, HIGH);
//        }
//    } else {
//      Serial.println("Z axis: No tremor detected!"); 
//      i++;
//    }
//
//    if (i==3)
//        digitalWrite(LED_BUILTIN, HIGH);

    // stop here
//    while (1);

    digitalWrite(LED_PIN, blinkState);
}

void SquareArray(double *vData, uint16_t bufferSize) {
  for (uint16_t i = 0; i < bufferSize; i++)  {
    vData[i] = vData[i]*vData[i];
  }
}
