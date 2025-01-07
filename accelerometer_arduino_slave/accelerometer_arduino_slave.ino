#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#include "arduinoFFT.h"

//arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
//const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
//const double samplingFrequency = 538;
//double vReal[samples];
//double vImag[samples];
//int fftIndex = 0;

//float data_array_x[100];
//float data_array_y[100];
//float data_array_z[100];
unsigned long lastTimestamp;
unsigned long counter;

/* Assign a unique ID to this sensor at the same time */
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// SPI mode
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(0xE5);
// I2C mode
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(0xE5);
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(10, 11, 12, 13, 0xE5);
char outputChar[50];

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

void setup(void) 
{
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(1000000);
//  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  
  accel.setRange(ADXL345_RANGE_16_G);
//   accel.setRange(ADXL345_RANGE_8_G);
//   accel.setRange(ADXL345_RANGE_4_G); 
//   accel.setRange(ADXL345_RANGE_2_G);
  
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);
  //default = ADXL345_DATARATE_0_10_HZ

  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0x80);

  pinMode(9, INPUT);
  
  /* Display some basic information on this sensor */
//  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
//  displayDataRate();
//  displayRange();
//  Serial.println(""); 
  counter = 0;
  lastTimestamp = millis();
//  Serial.println(lastTimestamp);

//  for(int i=0; i<samples; ++i){
//    vReal[i] = 0;
//    vImag[i] = 0;
//  }

//  Serial.print("register 0x38 = ");
//  Serial.println(accel.readRegister(0x38));
}

void loop(void) 
{
  /* Get a new sensor event */ 
  while(true){
    if(digitalRead(9) == 1){
      sensors_event_t event;
      unsigned long eventTime = millis();
      accel.getEventXYZ(&event);
//      accel.readRegister(ADXL345_REG_DATAX0);

      String outputString = "{\"T\":"+String(eventTime)+",\"X\":"+String(event.acceleration.x, 2)+",\"Y\":"+String(event.acceleration.y, 2)+",\"Z\":"+String(event.acceleration.z, 2)+"}\n";
      memset(outputChar, 0, sizeof(char)*50);
      outputString.toCharArray(outputChar, outputString.length()+1);
      Serial.write(outputChar, 50);   
  
//      counter++;
//      unsigned long curTime = millis();
//      if(curTime-lastTimestamp >= 1000){
//      //  if(counter == 1000){
//        Serial.print("counter = ");
//        Serial.println(counter);
//        counter = 0;
//        lastTimestamp = curTime;
//        Serial.println(curTime);
//      }
//      break;
    }
  }

//  data_array_x[counter] = event.acceleration.x;
//  data_array_y[counter] = event.acceleration.y;
//  data_array_z[counter] = event.acceleration.z;

//  if(counter == 100){
//    for(int i=0; i<100; ++i){
//        Serial.print("{\"X\": "); Serial.print(data_array_x[i]); Serial.print(",  ");
//        Serial.print("\"Y\": "); Serial.print(data_array_y[i]); Serial.print(",  ");
//        Serial.print("\"Z\": "); Serial.print(data_array_z[i]); Serial.print("}");
//        Serial.println("");
//    }
//    counter = 0;
//  }

//  if(fftIndex < samples){
//    vReal[fftIndex] = event.acceleration.z;
//    fftIndex++;
//  }
//  else{//reach sample size, calculate fft and clear array
//    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
//    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
//    double peakFreq = FFT.MajorPeak(vReal, samples, samplingFrequency);
//    Serial.print("major freq = ");
//    Serial.println(peakFreq, 4);
//    fftIndex = 0;
//  }

  

  // b'X: 9.77  Y: 0.08  Z: -2.43  m/s^2 \r\n'
 
  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("{\"X\": "); Serial.print(event.acceleration.x); Serial.print(",  ");
//  Serial.print("\"Y\": "); Serial.print(event.acceleration.y); Serial.print(",  ");
//  Serial.print("\"Z\": "); Serial.print(event.acceleration.z); Serial.print("}");
//  Serial.println("");
//  String outputString = "{\"X\":"+String(event.acceleration.x, 2)+",\"Y\":"+String(event.acceleration.y, 2)+",\"Z\":"+String(event.acceleration.z, 2)+"}\n";
//  char outputChar[35];
//  outputString.toCharArray(outputChar, 35);
//  Serial.write(outputChar, 35);
}
