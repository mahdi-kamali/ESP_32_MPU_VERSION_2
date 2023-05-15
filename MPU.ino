void startMPU() {


  int connectionTry = 3;
  long sensorCheckTimeDelay = 200 ;
  while (connectionTry != 0) {
    if (!mpu.begin(0x69, &I2Cone, 1)) {
      Serial.println("Failed to find MPU6050 chip ( SENSOR 1 )  Try => " + String(connectionTry));
      delay(sensorCheckTimeDelay);
      connectionTry--;
    } else {
      Serial.println(" ---------- MPU6050 chip ( SENSOR 1 ) FINDED !  ");
      sensorOneActive = true ;
      break;
    }
  }

  connectionTry = 3;
  while (connectionTry != 0) {
    if (!mpu2.begin(0x68, &I2Cone, 2)) {
      Serial.println("Failed to find MPU6050 chip ( SENSOR 2 )  Try => " + String(connectionTry));
      delay(sensorCheckTimeDelay);
      connectionTry--;
    } else {
      Serial.println(" ---------- MPU6050 chip ( SENSOR 2 ) FINDED !  ");
      sensorTwoActive = true ;
      break;
    }
  }


  connectionTry = 3;
  while (connectionTry != 0) {
    if (!mpu3.begin(0x68, &I2Ctwo, 3)) {
      Serial.println("Failed to find MPU6050 chip ( SENSOR 3 )  Try => " + String(connectionTry));
      delay(sensorCheckTimeDelay);
      connectionTry--;
    } else {
      Serial.println(" ---------- MPU6050 chip ( SENSOR 3 ) FINDED !  ");
      sensorThreeActive = true ;

      break;
    }
  }


  connectionTry = 3;
  while (connectionTry != 0) {
    if (!mpu4.begin(0x69, &I2Ctwo, 4)) {
      Serial.println("Failed to find MPU6050 chip ( SENSOR 4 )  Try => " + String(connectionTry));
      delay(sensorCheckTimeDelay);
      connectionTry--;
    } else {
      Serial.println(" ---------- MPU6050 chip ( SENSOR 4 ) FINDED !  ");
      sensorFourActive = true ;
      break;
    }
  }


}





void printMpuData(String sensorName, sensors_event_t accel_event, sensors_event_t gyro_event, sensors_event_t temp_event, long delayMillis) {


  sensorName = "/" + sensorName + ".csv";


  // dataFile = SD.open(sensorName, FILE_APPEND);



  // if (dataFile) {

  //   dataFile.print(sensorName);
  //   dataFile.print(",");
  //   dataFile.print(now.year());
  //   dataFile.print("-");
  //   dataFile.print(now.month());
  //   dataFile.print("-");
  //   dataFile.print(now.day());
  //   dataFile.print(",");
  //   dataFile.print(now.hour());
  //   dataFile.print(":");
  //   dataFile.print(now.minute());
  //   dataFile.print(":");
  //   dataFile.print(now.second());
  //   dataFile.print(",");
  //   dataFile.print(accel_event.acceleration.x);
  //   dataFile.print(",");
  //   dataFile.print(accel_event.acceleration.y);
  //   dataFile.print(",");
  //   dataFile.print(accel_event.acceleration.z);
  //   dataFile.print(",");
  //   dataFile.print(gyro_event.gyro.x);
  //   dataFile.print(",");
  //   dataFile.print(gyro_event.gyro.y);
  //   dataFile.print(",");
  //   dataFile.print(gyro_event.gyro.z);
  //   dataFile.print(",");
  //   dataFile.print(temp_event.temperature);
  //   dataFile.println();


  //   dataFile.close();
  // } else {
  //   Serial.println("Error !");
  // }






  Serial.print(sensorName);
  Serial.print("Current time: ");
  Serial.print(now.year());
  Serial.print("-");
  Serial.print(now.month());
  Serial.print("-");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  Serial.print(",");
  Serial.print("Accelerometer (m/s^2): x = ");
  Serial.print(accel_event.acceleration.x);
  Serial.print(", y = ");
  Serial.print(accel_event.acceleration.y);
  Serial.print(", z = ");
  Serial.print(accel_event.acceleration.z);
  Serial.print(", ");
  Serial.print("Gyroscope (deg/s): x = ");
  Serial.print(gyro_event.gyro.x);
  Serial.print(", y = ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(", z = ");
  Serial.print(gyro_event.gyro.z);
  Serial.print(", ");
  Serial.print("Temperature (°C): ");
  Serial.print(temp_event.temperature);  // اولی
  // Serial.print((temp_event.temperature -32) * 5/9  );
  Serial.println(" C");
}




void saveToSDCard(String sensorName, double ax, double ay, double az, double gx, double gy, double gz, sensors_event_t temp_event) {
  sensorName = "/" + sensorName + ".csv";


  dataFile = SD.open(sensorName, FILE_APPEND);


  DateTime currentTime = getUpdatedTime();



  if (dataFile) {

    dataFile.print(sensorName);
    dataFile.print(",");
    dataFile.print(currentTime.year());
    dataFile.print("-");
    dataFile.print(currentTime.month());
    dataFile.print("-");
    dataFile.print(currentTime.day());
    dataFile.print(",");
    dataFile.print(currentTime.hour());
    dataFile.print(":");
    dataFile.print(currentTime.minute());
    dataFile.print(":");
    dataFile.print(currentTime.second());
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.print(gz);
    dataFile.print(",");
    dataFile.print(temp_event.temperature);
    dataFile.println();


    dataFile.close();
  } else {
    Serial.println("Error !");
  }
}