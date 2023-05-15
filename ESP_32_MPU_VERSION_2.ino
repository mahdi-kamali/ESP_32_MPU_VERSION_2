#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <FastLED.h>


#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "RTClib.h"
#include <arduinoFFT.h>

#include "SD.h"











// I2C Wires #1
#define SDA_1 21
#define SCL_1 22



// I2C Wires #2
#define SDA_2 33
#define SCL_2 32




// Wifi Connection
const char* ssid = "100";
const char* password = "mahdihastam";





// Sd Card File Object
File dataFile;



// Sensors
Adafruit_MPU6050 mpu, mpu2, mpu3, mpu4;



// I2C Wires Objects
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);





// RTC ( Time )  Tehran UTC
const char* ntpServer = "pool.ntp.org";
const long utcOffsetInSeconds = 12600;  // UTC offset for Tehran



// Wifi For Time Update
WiFiUDP ntpUDP;
RTC_DS3231 rtc;

// Time Client Object
NTPClient timeClient(ntpUDP, ntpServer, utcOffsetInSeconds);
DateTime now;




// SD CART Cheap Select Pin
const int SD_CHIP_SELECT = 5;



const long MPU_TIME_DELAY = 0;
const long TIME_UPDATE_DELAY = 0;






// FFT CONSTS
#define SAMPLES 32         // Must be a power of 2
#define SAMPLING_FREQ 300  // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.




// FFT Values
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned int sampling_period_us;


// FFT Object
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);


// FFT Results Values
float frequency = 0;
float frequencyValue = 0;



// Sensor Object
class SensorFFT {
public:
  float get_AX() const {
    return AX;
  }

  void set_AX(float new_AX) {
    AX = new_AX;
  }

  float get_AY() const {
    return AY;
  }

  void set_AY(float new_AY) {
    AY = new_AY;
  }

  float get_AZ() const {
    return AZ;
  }

  void set_AZ(float new_AZ) {
    AZ = new_AZ;
  }

  float get_GX() const {
    return GX;
  }

  void set_GX(float new_GX) {
    GX = new_GX;
  }

  float get_GY() const {
    return GY;
  }

  void set_GY(float new_GY) {
    GY = new_GY;
  }

  float get_GZ() const {
    return GZ;
  }

  void set_GZ(float new_GZ) {
    GZ = new_GZ;
  }

private:
  float AX = 0;
  float AY = 0;
  float AZ = 0;
  float GX = 0;
  float GY = 0;
  float GZ = 0;
};



// Sensors Instances
SensorFFT sensorOneFFT;
SensorFFT sensorTwoFFT;
SensorFFT sensorThreeFFT;
SensorFFT sensorFourFFT;

bool sensorOneActive = false;
bool sensorTwoActive = false;
bool sensorThreeActive = false;
bool sensorFourActive = false;






void getMpuOneData() {


  // Sensor 1   AX




  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_AX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_AX(frequency);

  frequency = 0;
  frequencyValue = 0;






  // Sensor 1   AY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_AY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_AY(frequency);

  frequency = 0;
  frequencyValue = 0;





  // Sensor 1   AZ



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_AZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_AZ(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 1   GX



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = gyro_event.gyro.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_GX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_GX(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 1   GY



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = gyro_event.gyro.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_GY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_GY(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 1   GZ



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = gyro_event.gyro.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_1_GZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorOneFFT.set_GZ(frequency);

  frequency = 0;
  frequencyValue = 0;
}

void getMpuTwoData() {
  // Sensor 2   AX



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_AX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorTwoFFT.set_AX(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 2   AY



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_AY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");

  sensorTwoFFT.set_AY(frequency);


  frequency = 0;
  frequencyValue = 0;



  // Sensor 2   AZ



  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = accel_event.acceleration.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_AZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorTwoFFT.set_AZ(frequency);


  frequency = 0;
  frequencyValue = 0;


  // Sensor 2   GX


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    vReal[i] = gyro_event.gyro.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_GX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorTwoFFT.set_GX(frequency);


  frequency = 0;
  frequencyValue = 0;







  // Sensor 2   GY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_GY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorTwoFFT.set_GY(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 2   GZ


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_2_GZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorTwoFFT.set_GZ(frequency);

  frequency = 0;
  frequencyValue = 0;
}

void getMpuThreeData() {
  // Sensor 3   AX


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_AX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorThreeFFT.set_AX(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 3   AY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_AY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorThreeFFT.set_AY(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 3   AZ


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_AZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");

  sensorThreeFFT.set_AZ(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 3   GX


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_GX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorThreeFFT.set_GX(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 3   GY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_GY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorThreeFFT.set_GY(frequency);


  frequency = 0;
  frequencyValue = 0;



  // Sensor 3   GZ


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_3_GZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorThreeFFT.set_GZ(frequency);


  frequency = 0;
  frequencyValue = 0;
}

void getMpuFourData() {







  // Sensor 4   AX


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_AX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_AX(frequency);

  frequency = 0;
  frequencyValue = 0;



  // Sensor 4   AY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_AY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_AY(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 4   AZ


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = accel_event.acceleration.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_AZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_AZ(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 4   GX


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.x;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_GX(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_GX(frequency);

  frequency = 0;
  frequencyValue = 0;


  // Sensor 4   GY


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.y;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_GY(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_GY(frequency);

  frequency = 0;
  frequencyValue = 0;




  // Sensor 4   GZ


  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu2.getEvent(&accel_event, &gyro_event, &temp_event);

    // Serial.print(",");
    vReal[i] = gyro_event.gyro.z;
    vImag[i] = 0;
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();




  for (int i = 0; i < (SAMPLES / 2); i++) {

    if (vReal[i] > frequencyValue) {
      frequencyValue = vReal[i];
      frequency = i * (float)SAMPLING_FREQ / SAMPLES;
    }
  }

  Serial.print("S_4_GZ(HZ):");
  Serial.print(frequency, 1);  // print frequency with 1 decimal place
  Serial.println("");
  sensorFourFFT.set_GZ(frequency);


  frequency = 0;
  frequencyValue = 0;
}


void saveAllDataToSdCart() {



  // سنسور اولی
  if (sensorOneActive) {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu.getEvent(&accel_event, &gyro_event, &temp_event);
    // printMpuData("Sensor_1", accel_event, gyro_event, temp_event, MPU_TIME_DELAY);
    saveToSDCard("Sensor_1", sensorOneFFT.get_AX(), sensorOneFFT.get_AY(), sensorOneFFT.get_AZ(), sensorOneFFT.get_GX(), sensorOneFFT.get_GY(), sensorOneFFT.get_AZ(), temp_event);
  }




  // سنسور دومی
  if (sensorTwoActive) {
    sensors_event_t accel_event2, gyro_event2, temp_event2;
    mpu2.getEvent(&accel_event2, &gyro_event2, &temp_event2);
    // printMpuData("Sensor_2", accel_event2, gyro_event2, temp_event2, MPU_TIME_DELAY);
    saveToSDCard("Sensor_2", sensorTwoFFT.get_AX(), sensorTwoFFT.get_AY(), sensorTwoFFT.get_AZ(), sensorTwoFFT.get_GX(), sensorTwoFFT.get_GY(), sensorTwoFFT.get_AZ(), temp_event2);
  }





  // سنسور سومی
  if (sensorThreeActive) {
    sensors_event_t accel_event3, gyro_event3, temp_event3;
    mpu3.getEvent(&accel_event3, &gyro_event3, &temp_event3);
    // printMpuData("Sensor_3", accel_event3, gyro_event3, temp_event3, MPU_TIME_DELAY);
    saveToSDCard("Sensor_3", sensorThreeFFT.get_AX(), sensorThreeFFT.get_AY(), sensorThreeFFT.get_AZ(), sensorThreeFFT.get_GX(), sensorThreeFFT.get_GY(), sensorThreeFFT.get_AZ(), temp_event3);
  }



  // سنسور چهارمی
  if (sensorFourActive) {
    sensors_event_t accel_event4, gyro_event4, temp_event4;
    mpu4.getEvent(&accel_event4, &gyro_event4, &temp_event4);
    // printMpuData("Sensor_4", accel_event4, gyro_event4, temp_event4, MPU_TIME_DELAY);
    saveToSDCard("Sensor_4", sensorFourFFT.get_AX(), sensorFourFFT.get_AY(), sensorFourFFT.get_AZ(), sensorFourFFT.get_GX(), sensorFourFFT.get_GY(), sensorFourFFT.get_AZ(), temp_event4);
  }
}



TaskHandle_t Task2;


void setup() {
  Serial.begin(115200);
  initI2CBus();
  initWiFi();
  startMPU();
  initTime();
  initSdCart();
  getUpdatedTime();
  sampling_period_us = 10;
  pinMode(2, OUTPUT);
  Serial.print("loop() running on core ");
  Serial.println(xPortGetCoreID());
  delay(2000);
}




void loop() {



  if (sensorOneActive) {
    getMpuOneData();
  }
  if (sensorTwoActive) {
    getMpuTwoData();
  }
  if (sensorThreeActive) {
    getMpuThreeData();
  }
  if (sensorFourActive) {
    getMpuFourData();
  }


  saveAllDataToSdCart();


  // تست درگاه سنسور ها
  // scanI2COne();
  // scanI2CTwo();
}


