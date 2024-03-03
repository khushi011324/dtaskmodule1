const int motionSensor = 2;
const int buzzer = 7;
const int tiltSensor = 3;
const int motionSensor2 = 4;

int redLed = 13;
int redLed2 = 11;
const int redLed3 = PB4;

uint8_t motionState, tiltState, motionState2 = 0;

uint8_t premotion, premotion1, premotion2 = 0;

const uint16_t timer = 0;
const uint16_t compare = 31250;

void setup() {
  Serial.begin(9600);

  pinMode(buzzer, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(redLed2, OUTPUT);
  pinMode(tiltSensor, INPUT_PULLUP);
  pinMode(motionSensor, INPUT);
  pinMode(motionSensor2, INPUT);
  DDRB |= (1 << redLed3);

  PCICR |= 0b00000111;
  PCMSK2 |= 0b10011100;

  TCCR1A = 0;
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = timer;
  OCR1A = compare;

  TIMSK1 = (1 << OCIE1A);
  sei();
}

void loop() {
  if (premotion != motionState) {
    digitalWrite(redLed, motionState);
    Serial.println("Motion Sensor 1 Interrupt - Turned Red LED");
    premotion = motionState;
  }

  if (premotion1 != motionState2) {
    digitalWrite(redLed2, motionState2);
    Serial.println("Motion Sensor 2 Interrupt - Turned Red LED");
    premotion1 = motionState2;
  }

  if (premotion2 != tiltState) {
    digitalWrite(buzzer, tiltState);
    Serial.println("Tilt Sensor Interrupt - Buzzed");
    premotion2 = tiltState;
  }

  delay(200);
}

ISR(PCINT2_vect) {
  motionState = PIND & B00010000;
  motionState2 = PIND & B00000100;
  tiltState = PIND & B00001000;
}

ISR(TIMER1_COMPA_vect) {
  TCNT1 = timer;
  PORTB ^= (1 << redLed3);
}
