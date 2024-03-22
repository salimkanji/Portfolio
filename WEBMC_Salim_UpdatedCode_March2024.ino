/******************************************************************
  @file       nano33BLErev2.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              BMI270/BMM150 IMUs on the Nano 33 BLE Sense rev2.
  @author     David Such: Updated- Salim Kanji
  @copyright  Please see the accompanying LICENSE file.

  This sketch is configured to work with the MADGWICK, MAHONY,
  CLASSIC, COMPLEMENTARY, KALMAN & NONE Sensor Fusion options. Set the 
  algorithm that you wish to use with:

  imu.setFusionAlgorithm(SensorFusion::MADGWICK);

******************************************************************/

#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>
int enable = 0;

ReefwingAHRS ahrs;
SensorData data;

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

void setup() {
  //  Initialise the AHRS
  //  Use default fusion algo and parameters
  ahrs.begin();
  
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(-8.89);                      //  London, ON

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (IMU.begin() && ahrs.getBoardType() == BoardType::NANO33BLE_SENSE_R2) {
    Serial.println("BMI270 & BMM150 IMUs Connected."); 
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Gyroscope in degrees/second");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.print("Magnetic field sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Magnetic Field in uT");

    Serial.println ("[0] - Zero the Accelerometer, Gyroscope and Magnetometer");
    


  } 
  else {
    Serial.println("BMI270 & BMM150 IMUs Not Detected.");
    while(1);
  }
}

float roll_zero = 0;
float pitch_zero = 0;
float yaw_zero = 0;
float heading_zero = 0;

void loop() {
  if (IMU.gyroscopeAvailable()) {  IMU.readGyroscope(data.gx, data.gy, data.gz);  }
  if (IMU.accelerationAvailable()) {  IMU.readAcceleration(data.ax, data.ay, data.az);  }
  if (IMU.magneticFieldAvailable()) {  IMU.readMagneticField(data.mx, data.my, data.mz);  }

  ahrs.setData(data);
  ahrs.update();

  if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.

      


      char input = Serial.read();

      if (input == '0'){

      roll_zero = ahrs.angles.roll;
      pitch_zero = ahrs.angles.pitch;
      yaw_zero = ahrs.angles.yaw;
      heading_zero = ahrs.angles.heading;

       }

      float roll = ahrs.angles.roll - roll_zero;
      float pitch = ahrs.angles.pitch - pitch_zero;
      float yaw = ahrs.angles.yaw - yaw_zero;
      float heading = ahrs.angles.heading - heading_zero;

       

      
      Serial.print("--> Roll: ");
      Serial.print(roll, 2);
      Serial.print("\tPitch: ");
      Serial.print(pitch, 2);
      Serial.print("\tYaw: ");
      Serial.print(yaw, 2);
      Serial.print("\tHeading: ");
      Serial.print(heading, 2);
      Serial.print("\tLoop Frequency: ");
      Serial.print(loopFrequency);
      Serial.println(" Hz");

      loopFrequency = 0;
      previousMillis = millis();
    

    
    
  }

  loopFrequency++;
}
