#include <Servo.h>
#include <stdint.h>
#include "queue.h" // list (3) from sys/queue.h
#include "cal.h"

#define PIN_SERVO 10
#define PIN_IR A0
#define MEDIAN_NUM_SAMPLES 7

#define SENSOR_INTERVAL 10
#define SERVO_INTERVAL 5
#define SERIAL_INTERVAL 100

#define ALPHA 0.19
#define SERVO_ACCEL 1

float ir_distance();

inline float ema_filter(float val);
inline float median_filter(float val);

int getStopDistance(int spd);

LIST_HEAD(sample_head, sample_entry) sample_head;

struct sample_entry {
	float val;
	int age;
	LIST_ENTRY(sample_entry) sample_entries;
};

struct sample_entry samples[MEDIAN_NUM_SAMPLES];

Servo ser;

const float kp = 0.73;
const float kd = 71.0;
const float ki = 0.002;

unsigned long serial_last, servo_last, sensors_last;
float dist_ir, dist_ema, dist_median;
float volt, volt_median, volt_ema;

int duty_curr = 1210;
float err_curr, err_prev;
float pterm, dterm, iterm;
int duty_target;
int servo_speed;

void setup()
{
	ser.attach(PIN_SERVO);
	ser.writeMicroseconds(1610);
	Serial.begin(57600);
	serial_last = servo_last = sensors_last = 0;
	servo_speed = 0;

	sample_head = SLIST_HEAD_INITIALIZER(sample_head);
	LIST_INIT(&sample_head);
	for (int i = MEDIAN_NUM_SAMPLES - 1; i >= 0; i--) {
		samples[i].val = 255;
		samples[i].age = i;
		LIST_INSERT_HEAD(&sample_head, &samples[i], sample_entries);
	}
	for (int i = 0; i < MEDIAN_NUM_SAMPLES; i++) {
		volt = (float)analogRead(PIN_IR);
		volt_median = median_filter(volt);
		volt_ema = ema_filter(volt);
	}
}


void loop()
{
	if (millis() - sensors_last > SENSOR_INTERVAL) {
		sensors_last += SENSOR_INTERVAL;

		volt = (float)analogRead(PIN_IR);
		volt_median = median_filter(volt);
		volt_ema = ema_filter(volt_median);
		dist_ema = getIRDistance(volt_ema);

		err_curr = dist_ema - 255.0;
		pterm = kp * err_curr;
		dterm = kd * (err_curr - err_prev) * 1.0;
		if (dterm > 400)
			dterm = 400;
		if (dterm < -400)
			dterm = -400;
		iterm += ki * err_curr;
		err_prev = err_curr;
		duty_target = getServoDuty(pterm + dterm + iterm);
	}


	if (millis() - servo_last > SERVO_INTERVAL) {
		servo_last += SERVO_INTERVAL;
		if (duty_target > duty_curr &&
				duty_target - duty_curr <= getStopDistance(servo_speed) + servo_speed) {
			servo_speed -= SERVO_ACCEL;
		} else if (duty_target < duty_curr &&
				duty_target - duty_curr >= getStopDistance(servo_speed) + servo_speed) {
			servo_speed += SERVO_ACCEL;
		} else if (duty_target > duty_curr) {
			servo_speed += SERVO_ACCEL;
		} else {
			servo_speed -= SERVO_ACCEL;
		}
		servo_speed = constrain(servo_speed, -10, 10);

		duty_curr += servo_speed;
		ser.writeMicroseconds(duty_curr);
	}

	if (millis() - serial_last > SENSOR_INTERVAL) {
		serial_last += SENSOR_INTERVAL;
		Serial.print(",IR:");
		Serial.print(dist_ema);
		Serial.print(",T:");
		Serial.print(255);
		Serial.print(",P:");
		Serial.print(map(pterm,-500,500,510,610));
		Serial.print(",D:");
		Serial.print(map(dterm,-500,500,510,610));
		Serial.print(",I:");
		Serial.print(map(iterm,-500,500,510,610));
		Serial.print(",DTT:");
		Serial.print(map(duty_target,1000,2000,410,510));
		Serial.print(",DTC:");
		Serial.print(map(duty_curr,1000,2000,410,510));
		Serial.println(",-G:245,+G:265,m:0,M:800");
	}
}


inline float ir_distance()
{
	float val;
	float volt = float(analogRead(PIN_IR));
	return getIRDistance(volt);
	val = 63158.4/(volt-58.2)-8;
	return val;
}

inline float ema_filter(float val) {
	static bool initialized = false;
	static float val_ema = 0;
	if (! initialized) {
		initialized = true;
		val_ema = val;
		return val;
	}
	val_ema = ALPHA * val + (1.0 - ALPHA) * val_ema;
	return val_ema;
}


inline float median_filter(float val)
{
	struct sample_entry *ptr, *freeptr;

	LIST_FOREACH(ptr, &sample_head, sample_entries) { // update age and delete oldest.
		ptr->age++;
		if (ptr->age >= MEDIAN_NUM_SAMPLES) {
			freeptr = ptr;
			LIST_REMOVE(ptr, sample_entries);
		}
	}

	if (val <= LIST_FIRST(&sample_head)->val) { // insert new value in the middle.
		freeptr->val = val;
		freeptr->age = 0;
		LIST_INSERT_HEAD(&sample_head, freeptr, sample_entries);
	} else
		LIST_FOREACH(ptr, &sample_head, sample_entries) {
			if ((LIST_NEXT(ptr, sample_entries) == NULL)
					|| (ptr->val < val && val <= LIST_NEXT(ptr, sample_entries)->val)) {
				freeptr->val = val;
				freeptr->age = 0;
				freeptr->age = 0;
				LIST_INSERT_AFTER(ptr, freeptr, sample_entries);
				break;
			}
		}

	int idx = 0;
	LIST_FOREACH(ptr, &sample_head, sample_entries)
		if (idx++ >= (MEDIAN_NUM_SAMPLES / 2))
			return ptr->val;

	return  -1; // program shouldn't reach here.
}

int getStopDistance(int speed)
{
	int n, r, ret, spd;
	spd = abs(speed);
	n = spd / SERVO_ACCEL;
	r = spd % SERVO_ACCEL;
	ret = n * (n + 1) * SERVO_ACCEL / 2 + r * (n + 1);
	ret *= speed > 0 ? 1 : -1;
	return ret;
}
