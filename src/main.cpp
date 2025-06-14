#include <Arduino.h>

#define ENC_A     	(2)
#define ENC_B     	(3)
#define MD_A      	(6)
#define MD_B      	(7)
#define M_PWM     	(5)
#define END_STOP	(13)

#define PULSE_PER_REV 	(193.6f)
#define LOOP_DT    		(1)  // ms
#define DIST_PER_REV	(40)
#define DOWN_SAMPLE		(10)
#define VEL_FILT		(0.05f)
#define MIN_VEL			(4.9f)		// mm/s
#define MAX_VEL			(400.1f)	// mm/s
#define MIN_POS			(-0.1f)		// mm
#define MAX_POS			(200.1f)	// mm

#define REPORT_IDENT	{0xAA, 0xBB, 0xCC, 0xDD}
#define CTRL_CFG_IDENT	(0xEE)
#define PID_CFG_IDENT	(0xFF)

#define VELOCITY_MODE	(0)
#define POSITION_MODE	(1)

typedef struct {
	float vel;
	float pos;
	float target_vel;
	float target_pos;
	float output;
	float p;
	float i;
	float d;
	uint8_t ident[4];
}status_report_t;
status_report_t report = {0};

typedef struct {
	uint8_t ident[4];
	float target_vel;
	float target_pos;
	int32_t mode;
}control_t;

typedef struct {
	uint8_t ident[4];
	float kp;
	float ki;
	float kd;
	int32_t type; // 0: velocity, 1: position
}pid_config_t;
pid_config_t vel_cfg = {.ident = {0}, .kp = 10.0f, .ki = 0.1f, .kd = 0.0f, .type = 0};
pid_config_t pos_cfg = {.ident = {0}, .kp = 20.0f, .ki = 0.1f, .kd = 10.0f, .type = 1};

volatile int8_t isr_encoder = 0;
int32_t encoder = 0;
float vel = 0;	// mm/s
float pos = 0;	// mm
float target_vel = 0; // mm/s
float target_pos = 0; // mm
int32_t mode = VELOCITY_MODE; // 0: velocity, 1: position
float vel_filt = VEL_FILT;

void encoder_isr() {
	static volatile int phase = 0; // (AB) 0: 00, 1: 01, 2: 11, 3: 10
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
		isr_encoder ++;
	// 0 -> 3
	}else if(phase == 0 && new_phase == 3) {
		isr_encoder --;
	// 0 <-> 1 <-> 2 <-> 3
	}else {
		isr_encoder += new_phase - phase;
	}
	phase = new_phase;
}


float constrain_vel(float value) {
	if(value > 0 && value > MIN_VEL) {
		return constrain(value, MIN_VEL, MAX_VEL);
	}else if(value < 0 && value < -MIN_VEL) {
		return constrain(value, -MAX_VEL, -MIN_VEL);
	}else {
		return 0;
	}
}

float constrain_pos(float value) {
	if(value > 0 && value > MIN_POS) {
		return constrain(value, MIN_POS, MAX_POS);
	}else if(value < 0 && value < -MIN_POS) {
		return constrain(value, -MAX_POS, -MIN_POS);
	}else {
		return 0;
	}
}


void handle_serial() {
	static uint8_t serial_buffer[64] = {0};
	static int serial_cursor = 0;
	static int pack_type = 0; // 0: ctrl, 1: pid
	if(Serial.available()) {
		uint8_t n = Serial.read();
		serial_buffer[serial_cursor++] = n;
		// Serial.print(F("new: "));
		// Serial.println((char)n);
		// not match
		if((serial_cursor - 1 == 0 && n != 0xAA)
			|| (serial_cursor - 1 == 1 && n != 0xBB)
			|| (serial_cursor - 1 == 2 && n != 0xCC)
		) {
			serial_cursor = 0;
			// Serial.println(F("failed at first 3 char"));
		// already match 3 bytes, use last byte to get type
		}else if(serial_cursor - 1 == 3) {
			// control pack
			if(n == CTRL_CFG_IDENT) {
				pack_type = 0;
				// Serial.println(F("ctrl pack type"));
			// pid pack
			}else if(n == PID_CFG_IDENT) {
				pack_type = 1;
				// Serial.println(F("pid pack type"));
			// unknown type
			}else {
				serial_cursor = 0;
				// Serial.println(F("unknown pack type"));
			}
		// is ctrl pack
		}else if(pack_type == 0 && serial_cursor == sizeof(control_t)) {
			control_t *pack = (control_t*)&serial_buffer;
			mode = pack->mode;
			if(mode == VELOCITY_MODE) {
				target_vel = pack->target_vel;
				target_vel = constrain_vel(target_vel);
			}else if(mode == POSITION_MODE) {
				target_pos = pack->target_pos;
				target_pos = constrain_pos(target_pos);
			}
			serial_cursor = 0;
			// Serial.println(F("ctrl pack ok"));
		// is pid pack
		}else if(pack_type == 1 && serial_cursor == sizeof(pid_config_t)) {
			pid_config_t *pack = (pid_config_t*)&serial_buffer;
			if(pack->type == VELOCITY_MODE) {
				vel_cfg = *pack;
			}else if(pack->type == POSITION_MODE) {
				pos_cfg = *pack;
			}
			serial_cursor = 0;
			// Serial.println(F("pid pack ok"));
		}
	}
}


void drive_motor(int duty) {
	if(duty >= 0) {
  		digitalWrite(MD_B, 0);
		digitalWrite(MD_A, 1);
	}else {
		digitalWrite(MD_A, 0);
  		digitalWrite(MD_B, 1);
	}
	duty = constrain(duty, -255, 255);
	analogWrite(M_PWM, abs(duty));
	// Serial.println(duty);
}


void setup() {
  Serial.begin(921600);
  pinMode(MD_A, OUTPUT);
  pinMode(MD_B, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(END_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoder_isr, CHANGE);
//   drive_motor(-150);
//   delay(1000);
//   drive_motor(0);
}

void loop() {
	/* main control */
	static uint32_t last_update = 0;
	if(micros() - last_update >= (LOOP_DT * 1000)) {
		last_update = micros();
		/* 1 ms loop */
		int8_t delta_enc = isr_encoder;
		isr_encoder = 0;
		encoder += delta_enc;
		float this_vel = delta_enc * ((1000 / LOOP_DT) * (DIST_PER_REV / PULSE_PER_REV));
		// vel_filt = (100 - target_vel)
		vel = this_vel * vel_filt + vel * (1.0f - vel_filt);
		pos = encoder / (PULSE_PER_REV / DIST_PER_REV);
		/* position control */
		if(mode == POSITION_MODE) {
			static float pos_error_i = 0;
			static float last_pos_error = 0;
			float pos_error = target_pos - pos;
			pos_error_i += pos_error;
			pos_error_i = constrain(pos_error_i, -1000, 1000);
			float pos_p 	= pos_error * pos_cfg.kp;
			float pos_i 	= pos_error_i * pos_cfg.ki;
			float pos_d 	= (pos_error - last_pos_error) * pos_cfg.kd;
			target_vel = pos_p + pos_i + pos_d;
			target_vel = constrain_vel(target_vel);
			last_pos_error = pos_error;
		}
		/* velocity control */
		static float vel_error_i = 0;
		static float last_vel_error = 0;
		float vel_error = target_vel - vel;
		vel_error_i += vel_error;
		vel_error_i = constrain(vel_error_i, -1000, 1000);
		float vel_p 	= vel_error * vel_cfg.kp;
		float vel_i 	= vel_error_i * vel_cfg.ki;
		float vel_d 	= (vel_error - last_vel_error) * vel_cfg.kd;
		float vel_output = vel_p + vel_i + vel_d;
		last_vel_error = vel_error;
		drive_motor(vel_output);
		/* report */
		static int sample = 0;
		if(++sample >= DOWN_SAMPLE) {
			sample = 0;
			report = {
				.vel 		= vel,
				.pos 		= pos,
				.target_vel = target_vel,
				.target_pos = target_pos,
				.output 	= vel_output,
				.p 			= vel_p,
				.i 			= vel_i,
				.d			= 0,
				.ident		= REPORT_IDENT,
			};
			Serial.write((uint8_t*)&report, sizeof(report));
		}
	}
	/* endstop */
	if(digitalRead(END_STOP) == HIGH && target_vel < 0) {
		target_vel = 0;
		encoder = 0;
	}else if(pos > 200 && target_vel > 0){
		target_vel = 0;
	}
	/* serial */
	handle_serial();
	/* print */
	// static uint32_t last_print = 0;
	// if(millis() - last_print >= 500) {
	// 	last_print = millis();
	// 	Serial.print("pos: ");
	// 	Serial.print(pos);
	// 	Serial.print(", vel: ");
	// 	Serial.println(vel);
	// 	Serial.print("btn: ");
	// }
}
