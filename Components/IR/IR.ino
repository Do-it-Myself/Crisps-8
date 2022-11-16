#define IR_PIN A0
#define IR_THRESHOLD 775

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
    Serial.println(analogRead(irPin));
    return (analogRead(irPin) < threshold);
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
