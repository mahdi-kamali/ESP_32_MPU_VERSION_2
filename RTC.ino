


void initTime() {
  rtc.begin();
  timeClient.begin();
}



DateTime getUpdatedTime() {
  timeClient.update();
  DateTime now = DateTime(timeClient.getEpochTime());
  rtc.adjust(now);
  return  rtc.now();
}


