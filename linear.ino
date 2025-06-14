#define ENC_A     2
#define ENC_B     3
#define MD_A      4
#define MD_B      5
#define M_PWM     9

#define PULSE_PER_ROT 193.6f
#define ENC_BUFF_CNT  10
#define ENC_DT    3  // us

volatile int32_t encoder = 0;  
volatile int phase = 0; // (AB) 0: 00, 1: 01, 2: 11, 3: 10

void encoder_isr() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  int new_phase = -1;
  if(!a && !b) {
    new_phase = 0;
  }else if(!a && b) {
    new_phase = 1;
  }else if(a && b) {
    new_phase = 2;
  }else if(a && !b) {
    new_phase = 3;
  }
  // 3 -> 0
  if(phase == 3 && new_phase == 0) {
    encoder ++;
  // 0 -> 3
  }else if(phase == 0 && new_phase == 3) {
    encoder --;
  // 0 <-> 1 <-> 2 <-> 3
  }else {
    encoder += new_phase - phase;
  }
  phase = new_phase;
}


void setup() {
  Serial.begin(9600);
  // pinMode(MD_A, OUTPUT);
  // pinMode(MD_B, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(0, encoder_isr, CHANGE);
  attachInterrupt(1, encoder_isr, CHANGE);
  // q2
  // digitalWrite(MD_A, 1);
  // digitalWrite(MD_B, 0);
  // analogWrite(M_PWM, 200);
}

// int last_encoder = 0;
// int encoder_cursor = 0;
// uint32_t last_update = millis();

void loop() {
  // Serial.print("Encoder: ");
  // Serial.println(encoder);
  // delay(100);
  static int32_t last_encoder = 0;
  delay(1000);
  // get rpm
  float rotation = (encoder - last_encoder) / PULSE_PER_ROT;
  last_encoder = encoder;
  Serial.print("RPM: ");
  Serial.println(rotation * 60);
}
