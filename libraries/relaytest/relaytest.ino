
#define PIN_OFF 25
#define PIN_ON 26
bool stat = false;

void setup() {
  // put your setup code here, to run once:

  pinMode(PIN_ON, OUTPUT);
  pinMode(PIN_OFF, OUTPUT);
    SerialUSB.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (stat) {
       digitalWrite(PIN_ON, HIGH);
      digitalWrite(PIN_OFF, LOW);
      SerialUSB.println("On...");
      stat = false;
  }
  else{
      digitalWrite(PIN_ON, LOW);
      digitalWrite(PIN_OFF, HIGH);
      SerialUSB.println("Off...");
      stat = true;

  }

  delay(5000);
}
