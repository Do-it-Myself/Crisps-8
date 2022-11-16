#define IR_PIN A0
#define IR_THRESHOLD 780

class IR{
private:
  // class field
  int irPin;

public:
  // constructor
  IR(int pin) {
    pinMode(pin, INPUT);
    irPin = pin;
  }

  // functions
  bool obstacle(int threshold) {
    int analog = 0;
    int analog_reading;
    for (int i = 0; i < 10; i++) {
      analog_reading = analogRead(irPin);
      analog += analog_reading;
    }
    analog /= 10;
    Serial.println(analog);
    return (analog < threshold);
  }
};

IR irBlock(IR_PIN);

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(irBlock.obstacle(IR_THRESHOLD));
  delay(500);
}
