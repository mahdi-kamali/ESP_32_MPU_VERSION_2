void initSdCart() {
  Serial.print("Initializing SD card...");

  while (!SD.begin(SD_CHIP_SELECT)) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("SD card initialization failed!");
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println("SD card initialized.");



  String mpuHeader = "Sensor Name, Date, Time, AX, AY, AZ, GX, GY,GZ, Temperature (°C)";

  setDataCsvHeaders("/Sensor_1.csv ", mpuHeader);
  setDataCsvHeaders("/Sensor_2.csv ", mpuHeader);
  setDataCsvHeaders("/Sensor_3.csv ", mpuHeader);
  setDataCsvHeaders("/Sensor_4.csv ", mpuHeader);

  delay(2000);
}



void writeMpuOneData() {
}



void setDataCsvHeaders(String fileName, String fileHeaderString) {
  dataFile = SD.open(fileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to create file!");
    return;
  }
  dataFile.println(fileHeaderString);
  dataFile.close();
}


// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
// void appendFile(fs::FS &fs, const char * path, const char * message) {
//   Serial.printf("Appending to file: %s\n", path);

//   File file = fs.open(path, FILE_APPEND);
//   if(!file) {
//     Serial.println("Failed to open file for appending");
//     return;
//   }
//   if(file.print(message)) {
//     Serial.println("Message appended");
//   } else {
//     Serial.println("Append failed");
//   }
//   file.close();
// }
