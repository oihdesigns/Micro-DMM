/*
  Simple Digital I/O Configuration

  DIO 2: OUTPUT, always LOW
  DIO 3: OUTPUT, always HIGH
  DIO 4: INPUT
*/

const int PIN_LOW  = 2;
const int PIN_HIGH = 3;
const int PIN_INPUT = 4;

void setup() {
  // Configure outputs
  pinMode(PIN_LOW, OUTPUT);
  pinMode(PIN_HIGH, OUTPUT);

  // Set output states
  digitalWrite(PIN_LOW, LOW);
  digitalWrite(PIN_HIGH, HIGH);

  // Configure input
  pinMode(PIN_INPUT, INPUT);
}

void loop() {
  // Read the input if needed
  int inputState = digitalRead(PIN_INPUT);

  // Prevent compiler warnings if the value isn't used
  (void)inputState;

  // Nothing else to do
}