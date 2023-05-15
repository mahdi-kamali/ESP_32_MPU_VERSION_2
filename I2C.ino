

void initI2CBus() {
  I2Cone.begin(SDA_1, SCL_1, 400000);
  I2Ctwo.begin(SDA_2, SCL_2, 400000);
}


void scanI2COne() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning... I2CS 1");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    I2Cone.beginTransmission(address);
    error = I2Cone.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000);
}



void scanI2CTwo() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning... I2C 2 ");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    I2Ctwo.beginTransmission(address);
    error = I2Ctwo.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000);
}