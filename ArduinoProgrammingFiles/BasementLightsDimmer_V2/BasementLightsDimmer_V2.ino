#include <Wire.h>

// Define DS3502 I2C parameters
#define DS3502_ADDRESS 0x28   // Change this to your DS3502 I2C address if different
#define DS3502_CMD_WRITE 0x00 // Command used to write to the wiper register

// Global variables for D7 toggle functionality
bool LightsOn = LOW;     // Current state of D7 (LOW or HIGH)
int lastD8State = LOW;  // Previous reading from D8
int level = 3;

const int Button = 9;

// Toggle D7 and print its state to the Serial monitor.
void toggleD7() {
  LightsOn = !LightsOn;  // Invert state
  digitalWrite(7, LightsOn);
  Serial.print("D7 toggled to: ");
  Serial.println(LightsOn ? "HIGH" : "LOW");
}

// Set the DS3502 so that it provides a resistance equal to level*1000 ohms.
// Assumes a full range of 0-5000 ohms, mapping to a digital value 0-255.
void setLights(uint8_t level) {
  if (level < 1 || level > 5) {
    Serial.println("Invalid lights level. Please choose a value between 1 and 5.");
    return;
  }
  
  // Calculate desired resistance in ohms.
  uint16_t desiredResistance = level * 1000;  // For example: 1 -> 1000 ohm, 2 -> 2000 ohm, etc.
  
  // Map 0-5000 ohms to a value between 0 and 255.
  uint8_t potValue = map(desiredResistance, 0, 10000, 0, 255);
  
  // Write the calculated potentiometer value to DS3502 via I2C.
  Wire.beginTransmission(DS3502_ADDRESS);
  Wire.write(DS3502_CMD_WRITE); // Command to set the wiper (check your DS3502 datasheet)
  Wire.write(potValue);
  Wire.endTransmission();
  
  Serial.print("Set lights to ");
  Serial.print(desiredResistance);
  Serial.print(" ohms (digital value: ");
  Serial.print(potValue);
  Serial.println(")");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C communication
  
  // Configure D7 as output and D8 as input.
  pinMode(7, OUTPUT);
  pinMode(8, INPUT);
  pinMode(Button, INPUT_PULLUP);
  
  // Initialize D7 and record the initial state of D8.
  digitalWrite(7, LightsOn);
  lastD8State = digitalRead(8);
}

void loop() {
  // Process incoming serial commands.
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Toggle D7 if the command is "TOGGLE D7"
    if (command == "TOGGLE D7") {
      toggleD7();
    
    // Check if the command begins with "SET LIGHTS "
    } else if (command.startsWith("SET LIGHTS ")) {
      // Extract the numerical part after "SET LIGHTS ".
      String levelStr = command.substring(11);
      levelStr.trim();
      int level = levelStr.toInt();
      if (level == 0) { // toInt() returns 0 if the conversion fails
        Serial.println("Invalid lights level. Please send a number between 1 and 5.");
      } else {
        setLights((uint8_t)level);
      }
    }
  }
  
  // Monitor D8 for a state change (toggle D7 when D8 changes its state).
  int currentD8State = digitalRead(8);
  if(currentD8State != lastD8State){
    if(digitalRead(D7) == LOW){
      digitalWrite(D7, HIGH);
      level = 5;
      setLights((uint8_t)level);      
    }
    else if(digitalRead(D7) == HIGH){
      digitalWrite(D7, LOW);;
    }
    lastD8State = currentD8State;
        // A brief delay can help with debouncing if needed.
    delay(50);
    }

if(digitalRead(D9) == LOW){
  if(digitalRead(D7) == LOW){
    digitalWrite(D7, HIGH);
    level = 1;
    setLights((uint8_t)level);
  } else if (level == 1){
    level = 3;
    setLights((uint8_t)level);
    } else if (level == 3){
    level = 5;
    setLights((uint8_t)level);
  } else if (level == 5){
    digitalWrite(D7, LOW);

}
delay(500);
}

/*
  if (currentD8State != lastD8State) {
    toggleD7();    
    lastD8State = currentD8State;

  }
  */
}
