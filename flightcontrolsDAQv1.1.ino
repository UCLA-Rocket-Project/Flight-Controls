#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h> // 220 Hz
#include <Adafruit_L3GD20.h> // 190 or 380 Hz (we don't know but high frequency = less precision) 
#include <Adafruit_BMP280.h> // 157 Hz

#include <SD.h>
#define cardSelect 4 //default pin for communicating with the built in SD card board
File logfile;  //file to write to (object named logfile of type File)
const byte ledPin = 13;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_L3GD20 gyro;
Adafruit_BMP280 bmp;


/* Initialize Variables */
double a_x = 0, a_y = 0, a_z = 0,
    gy_x = 0, gy_y = 0, gy_z = 0,
    mag_x = 0, mag_y = 0, mag_z = 0,
    alt = 0;
    
void error(uint8_t errno) { //error number function, blinks out error number 
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(13, HIGH); //Pin 13 is built in LED
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

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

void setup(void)
{
  pinMode(ledPin, OUTPUT);    
  digitalWrite(ledPin, LOW);  
  Serial.begin(9600);
//  while(!Serial);
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //error(1);
    while(1);
  }
    if(!mag.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //error(2);
    //while(1); //this is always called, we don't know if it is because of physical error or library error
  }
    // Try to initialise and warn if we couldn't detect the chip
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }

  mag.setMagRate(LSM303_MAGRATE_220); // Sets mag to 220 Hz

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    //error(3);
    //while (1); // this is always called, we don't know if it is because of physical error or library error
  }
    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  /* WRITE REGISTRY FOR +-16gs MODE  */
  accel.write8pub(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0b00110000);
  Serial.print(accel.read8pub(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A));

  /* SD CARD WRITING SETUP    */
  if (!SD.begin(cardSelect)) {  //initialize card
    Serial.println("Card init. failed!");
    //error(4);
  }
  char filename[16];
  strcpy(filename, "IMUDAT00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    error(5);
  }
  logfile.println("time,a_x,a_y,a_z,gy_x,gy_y,gy_z,mag_x,mag_y,mag_z,alt");
  
  digitalWrite(13, HIGH) ;
}

sensors_event_t magEvent;
sensors_event_t accelEvent;
unsigned long startTime;
int flushCount = 0;
  
void loop()
{
  //TODO: DEBUG INFO
  /* Get a new sensor event */
  
  startTime = millis();

  accel.getEvent(&accelEvent);

//  a_x = accelEvent.acceleration.x;
//  a_y = accelEvent.acceleration.y;
//  a_z = accelEvent.acceleration.z;
  a_x = accel.raw.x;
  a_y = accel.raw.y;
  a_z = accel.raw.z;

  Serial.print("Time to read ACCEL: ");
  Serial.println(millis() - startTime);
  startTime = millis();

  gyro.read();
  gy_x = gyro.data.x;
  gy_y = gyro.data.y;
  gy_z = gyro.data.z;

  Serial.print("Time to read GYRO: ");
  Serial.println(millis() - startTime);
  startTime = millis();
  
  mag.getEvent(&magEvent);
  mag_x = magEvent.magnetic.x;
  mag_y = magEvent.magnetic.y;
  mag_z = magEvent.magnetic.z;

  Serial.print("Time to read MAG: ");
  Serial.println(millis() - startTime);
  startTime = millis();

  //bmp.readPressure();
  //bmp.readTemperature();
  alt = bmp.readAltitude(1013.25);

  Serial.print("Time to read ALT: ");
  Serial.println(millis() - startTime);
  startTime = millis();
  
  // Serial.print(millis());
  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print(a_x);
  // Serial.print(",");
  // Serial.print(a_y);
  // Serial.print(",");
  // Serial.print(a_z);
  // Serial.print(",");
  // /*  Degrees per second     */
  // Serial.print(gy_x);
  // Serial.print(",");
  // Serial.print(gy_y);
  // Serial.print(",");
  // Serial.print(gy_z);
  // Serial.print(",");
  // /*     microTesla (uT)     */
  // Serial.print(mag_x);
  // Serial.print(",");
  // Serial.print(mag_y);
  // Serial.print(",");
  // Serial.print(mag_z);
  // Serial.print(",");
  // Serial.print(alt);
  // Serial.print("\n");

  logfile.print(millis());
  logfile.print(",");
  logfile.print(a_x);
  logfile.print(",");
  logfile.print(a_y);
  logfile.print(",");
  logfile.print(a_z);
  logfile.print(",");
  /*  Degrees per second     */
  logfile.print(gy_x);
  logfile.print(",");
  logfile.print(gy_y);
  logfile.print(",");
  logfile.print(gy_z);
  logfile.print(",");
  /*     microTesla (uT)     */
  logfile.print(mag_x);
  logfile.print(",");
  logfile.print(mag_y);
  logfile.print(",");
  logfile.print(mag_z);
  logfile.print(",");
  logfile.println(alt);

  Serial.print("Time to WRITE ");
  Serial.println(millis() - startTime);
  startTime = millis();
  
  flushCount++;

  if (flushCount == 5){
    logfile.flush();  
    flushCount = 0;
  }
  

  Serial.print("Time to FLuSH ");
  Serial.println(millis() - startTime);
  startTime = millis();

  /* Delay before the next sample */
}
