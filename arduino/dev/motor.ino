const byte pwm_motor1 = 9;
const byte dir _motor1 = 11;

const byte pwm_motor2 = 10;
const byte dir_motor2 = 7;

void setup() {
    pinMode(pwm_motor1, OUTPUT);
    pinMode(dir_motor1, OUTPUT);
    pinMode(pwm_motor2, OUTPUT);
    pinMode(dir_motor2, OUTPUT);

    TCCR1B = (TCCR1B & 0b11111000) | 0x01;
}

void loop() {
    digitalWrite(dir_motor1, 1);;
    analogWrite(pwm_motor0, 255);

    digitalWrite(dir_motor2, 0);
    analogWrite(pwm_motor2, 255);
}
