#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  while (!Serial); 
}

void loop() {
  if (Serial.available() > 0) {
    // Expected Format: PINS:A0,A1|BITS:12|TIME:1000|SMOOTH:7|LOG:1000
    String command = Serial.readStringUntil('\n');
    
    int bitRes = command.substring(command.indexOf("BITS:") + 5, command.indexOf("|TIME")).toInt();
    int duration = command.substring(command.indexOf("TIME:") + 5, command.indexOf("|SMOOTH")).toInt();
    int smoothCount = command.substring(command.indexOf("SMOOTH:") + 7, command.indexOf("|LOG")).toInt();
    int logTarget = command.substring(command.indexOf("LOG:") + 4).toInt();
    
    if (smoothCount < 1) smoothCount = 1;
    if (logTarget < 1) logTarget = 1;
    
    String pinsPart = command.substring(5, command.indexOf("|BITS"));
    analogReadResolution(bitRes);
    
    int pins[12];
    String pinNames[12];
    uint32_t counts[12] = {0};
    double sums[12] = {0};
    double sqSums[12] = {0};
    int minVals[12], maxVals[12];
    
    // Logging buffers (using static to avoid stack overflow, limited to 2000 points per pin)
    // For Arduino Giga, we have plenty of RAM.
    const int MAX_LOG = 10000;
    if (logTarget > MAX_LOG) logTarget = MAX_LOG;
    static int logData[8][MAX_LOG]; 
    int loggedCount[8] = {0};

    int pinCount = 0;
    int commaIdx = 0;
    while (commaIdx != -1 && pinCount < 8) {
      int nextComma = pinsPart.indexOf(',', commaIdx);
      String pinName = (nextComma == -1) ? pinsPart.substring(commaIdx) : pinsPart.substring(commaIdx, nextComma);
      pinNames[pinCount] = pinName;
      
      if(pinName == "A0") pins[pinCount] = A0;
      else if(pinName == "A1") pins[pinCount] = A1;
      else if(pinName == "A2") pins[pinCount] = A2;
      else if(pinName == "A3") pins[pinCount] = A3;
      else if(pinName == "A4") pins[pinCount] = A4;
      else if(pinName == "A5") pins[pinCount] = A5;
      else if(pinName == "A6") pins[pinCount] = A6;
      else if(pinName == "A7") pins[pinCount] = A7;

      minVals[pinCount] = 65535;
      maxVals[pinCount] = 0;
      pinCount++;
      if (nextComma == -1) break;
      commaIdx = nextComma + 1;
    }

    uint32_t logIntervalUs = ((uint32_t)duration * 1000UL) / (uint32_t)logTarget;
    if (logIntervalUs < 1) logIntervalUs = 1;

    uint32_t startTime = millis();
    uint32_t nextLogUs = micros();

    while (millis() - startTime < (uint32_t)duration) {
      uint32_t nowUs = micros();
      bool shouldLog = (nowUs >= nextLogUs);
      if (shouldLog) nextLogUs += logIntervalUs;

      for (int i = 0; i < pinCount; i++) {
        long subSum = 0;
        for(int s = 0; s < smoothCount; s++) {
          subSum += analogRead(pins[i]);
        }
        int filteredVal = subSum / smoothCount;

        if (filteredVal < minVals[i]) minVals[i] = filteredVal;
        if (filteredVal > maxVals[i]) maxVals[i] = filteredVal;
        sums[i] += filteredVal;
        sqSums[i] += (double)filteredVal * filteredVal;
        counts[i]++;

        if (shouldLog && loggedCount[i] < logTarget) {
            logData[i][loggedCount[i]] = filteredVal;
            loggedCount[i]++;
        }
      }
    }

    // 1. Output Stats
    for (int i = 0; i < pinCount; i++) {
      double mean = sums[i] / counts[i];
      double rms = sqrt(sqSums[i] / counts[i]);
      double stdDev = sqrt(fabs((sqSums[i] / counts[i]) - (mean * mean)));

      Serial.print("PIN:"); Serial.print(pinNames[i]);
      Serial.print("|BITS:"); Serial.print(bitRes);
      Serial.print("|COUNT:"); Serial.print(counts[i]);
      Serial.print("|MIN:"); Serial.print(minVals[i]);
      Serial.print("|MAX:"); Serial.print(maxVals[i]);
      Serial.print("|MEAN:"); Serial.print(mean, 4);
      Serial.print("|RMS:"); Serial.print(rms, 4);
      Serial.print("|STD:"); Serial.println(stdDev, 4);
    }

    // 2. Output Logged Data for Plotting
    for (int i = 0; i < pinCount; i++) {
        Serial.print("DATA:"); Serial.print(pinNames[i]); Serial.print("|VALS:");
        for (int j = 0; j < loggedCount[i]; j++) {
            Serial.print(logData[i][j]);
            if (j < loggedCount[i] - 1) Serial.print(",");
        }
        Serial.println();
    }
    
    Serial.println("DONE");
  }
}