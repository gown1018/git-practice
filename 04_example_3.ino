int count = 0;
int toggle = LOW;

const int PIN_LED = LED_BUILTIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);

  Serial.println("Hello world");

  digitalWrite(PIN_LED, toggle);
}

void loop() {
  count++;
  Serial.println(count);

  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);

  delay(1000);
}

int toggle_state(int currentState) {
  return !currentState;
}
