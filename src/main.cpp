#include <Arduino.h>
#include <config.h>
#include <Servo.h>
#include <pid.h>
#include <kinematic.h>
#include <odometry.h>
#include <imu.h>
#include <USBHost_t36.h>

void launcher();
void moveBase();
void setMotor(int cwPin, int ccwPin, float pwmVal);
template <int j>
void readEncoder();
void linearGo(float speed_go, float speed_break);
void upperRobot();
void parseJoystickData(String data);
void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_);
void dribble_pneumatic();
void button_list(uint32_t buttons, int joy_axis);

USBHost usb_joy;
USBHub joy_hub(usb_joy);
USBHIDParser hid_joy(usb_joy);

#define COUNT_JOYSTICKS 4

JoystickController joy_control[COUNT_JOYSTICKS]{
	JoystickController(usb_joy), JoystickController(usb_joy),
	JoystickController(usb_joy), JoystickController(usb_joy)};
int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = {&joy_hub, &joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3], &hid_joy};
#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]", "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joy_control[0], &joy_control[1], &joy_control[2], &joy_control[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};
bool show_changed_only = false;

uint8_t joystick_left_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_right_trigger_value[COUNT_JOYSTICKS] = {0};
uint64_t joystick_full_notify_mask = (uint64_t)-1;

Servo esc_first;
Servo esc_second;

int sign;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prevT = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
volatile long pos[6];

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID launcher_up(0, 180, ESC_UP_KP, ESC_UP_KI, ESC_UP_KD);
PID launcher_down(0, 180, ESC_DOWN_KP, ESC_DOWN_KI, ESC_DOWN_KD);

Kinematic kinematic(
	Kinematic::LINO_BASE,
	MOTOR_MAX_RPS,
	MAX_RPS_RATIO,
	MOTOR_OPERATING_VOLTAGE,
	MOTOR_POWER_MAX_VOLTAGE,
	WHEEL_DIAMETER,
	ROBOT_DIAMETER);

Odometry odometry;

IMU imu_sensor;

void setup()
{
	// put your setup code here, to run once:
	// Serial8.begin(115200);
	Serial.begin(115200);
	usb_joy.begin();

	bno.begin();
	bno.setExtCrystalUse(true);

	esc_first.attach(esc_up, 1000, 2000);
	esc_second.attach(esc_down, 1000, 2000);

	pinMode(solenoid, OUTPUT);
	for (int i = 0; i < 4; i++)
	{
		pinMode(cw[i], OUTPUT);
		pinMode(ccw[i], OUTPUT);

		pinMode(catcher_ccw, OUTPUT);
		pinMode(catcher_cw, OUTPUT);

		analogWriteFrequency(cw[i], PWM_FREQUENCY);
		analogWriteFrequency(ccw[i], PWM_FREQUENCY);

		analogWriteFrequency(catcher_cw, PWM_FREQUENCY);
		analogWriteFrequency(catcher_ccw, PWM_FREQUENCY);

		analogWriteResolution(PWM_BITS);
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);

		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);
	}

	pinMode(prox_back, INPUT_PULLUP);
	pinMode(prox_front, INPUT_PULLUP);

	pinMode(prox_dribble, INPUT_PULLUP);

	pinMode(laser1, OUTPUT);
	pinMode(laser2, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);

}

int numbers[8]; // adjust size as needed
double value_upper_launcher = 0;
double value_lower_launcher = 0;
unsigned long launch_prevT = 0;
String inputString;

int applyDeadzone(int value, int deadzone = 10)
{
	if (abs(value) < deadzone)
	{
		return 0;
	}
	return value;
}

int cmd_to_dribble = 0;

void loop()
{
	usb_joy.Task();

	unsigned long launch_currT = micros();
	float launch_dt = ((float)(launch_currT - launch_prevT)) / 1.0e6;

	int trig_end_limit = digitalRead(prox_back);
	int trig_start_limit = digitalRead(prox_front);

	if ((trig_end_limit == 0 || trig_start_limit == 0) &&
		!(button.start == 1 || button.LT == 1))
	{
		setMotor(catcher_cw, catcher_ccw, 0);
	}

	if (joy_control[0].available())
	{
		uint32_t buttons = joy_control[0].getButtons();
		button_list(buttons, joy_control[0].getAxis(9));

		joystick.axis1_x = applyDeadzone(joy_control[0].getAxis(1) - 128);
		joystick.axis1_y = applyDeadzone(joy_control[0].getAxis(0) - 128);
		joystick.axis0_x = applyDeadzone(joy_control[0].getAxis(5) - 128);
		joystick.axis0_y = applyDeadzone(joy_control[0].getAxis(2) - 128);

		moveBase();
		dribble_pneumatic();
		launcher();
	}
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

	if (cmd_to_dribble == 4)
	{
		calculate_smooth_vel(value_upper_launcher, 135, launch_dt, 25.0);
		calculate_smooth_vel(value_lower_launcher, 180, launch_dt, 25.0);
	}
	else
	{
		calculate_smooth_vel(value_upper_launcher, 0, launch_dt, 25.0);
		calculate_smooth_vel(value_lower_launcher, 0, launch_dt, 25.0);
	}

	esc_first.write(value_upper_launcher);
	esc_second.write(value_lower_launcher);
	launch_prevT = launch_currT;
	Serial.print(event.orientation.x);
	Serial.println();
}

bool buttonPressed = false;
bool one_cycle = false;

void linearGo(float speed_go, float speed_break)
{
	int trig_back_limit = digitalRead(prox_back);
	int trig_front_limit = digitalRead(prox_front);

	if (speed_go > 0)
	{
		if (trig_front_limit == 0)
		{
		}
		else
		{
			setMotor(catcher_cw, catcher_ccw, speed_go);
		}
	}
	else if (speed_go < 0)
	{
		if (trig_back_limit == 0)
		{
		}
		else
		{
			setMotor(catcher_cw, catcher_ccw, speed_go);
		}
	}
	else
	{
		setMotor(catcher_cw, catcher_ccw, 0);
	}
}
bool dribble_once = false;

bool catching = false;
bool ballout = false;

void dribble_pneumatic()
{
	int trig_back_limit = digitalRead(prox_back);
	int trig_front_limit = digitalRead(prox_front);
	int prox_dribble_val = digitalRead(prox_dribble);

	if (button.RT == 1 && buttonPressed == false)
	{
		Serial.print(" | button RT | ");
		cmd_to_dribble = 1;
		buttonPressed = true;
	}
	else if (button.RT == 0 && buttonPressed == true)
	{
		buttonPressed = false;
	}
	if (button.LT == 1 && cmd_to_dribble == 4)
	{
		Serial.print(" | button Lt | ");
		digitalWrite(cylinder_side, 0);
		moveBase();
		delay(10);
		cmd_to_dribble = 0;
	}
	else if (cmd_to_dribble == 1)
	{
		linearGo(150, 0);
		Serial.print(" | cmd2dribble 1 | ");
		if (trig_front_limit == 0)
		{
			cmd_to_dribble = 2;
		}
	}
	else if (cmd_to_dribble == 2)
	{
		Serial.print(" | cmd2dribble 2 | ");
		if (trig_front_limit == 0)
		{
			if (dribble_once == false)
			{
				Serial.print(" | !dribble_once | ");
				digitalWrite(cylinder_upper, 1);
				moveBase();
				delay(20);
				digitalWrite(cylinder_side, 1);
				moveBase();
				delay(70);
				digitalWrite(cylinder_upper, 0);
				moveBase();
				delay(20);
				if (prox_dribble_val == 1)
				{
					dribble_once = true;
					catching = false;
				}
			}
			else if (prox_dribble_val == 0 && dribble_once == true && catching == false)
			{
				Serial.print(" | catching | ");
				digitalWrite(cylinder_side, 0);
				delay(20);

				// dribble_once = true;
				catching = true;
			}
			else if (catching == true)
			{
				linearGo(-128, 0);
			}
		}
		else if (catching == true && trig_back_limit == 1)
		{
			Serial.print(" | back | ");
			linearGo(-128, 0);
		}
		else if (trig_back_limit == 0 && catching == true)
		{
			dribble_once = false;
			digitalWrite(cylinder_side, 1);
			delay(20);
			moveBase();
			cmd_to_dribble = 3;
		}
	}
	else if (cmd_to_dribble == 3)
	{
		Serial.print(" | cmd2dribble 3 | ");
		linearGo(-128, 0);
		if (trig_back_limit == 0)
		{
			delay(200);
			digitalWrite(cylinder_side, 1);
			delay(20);
			cmd_to_dribble = 4;
		}
	}
}
const unsigned long pressDuration = 400;
const unsigned long releaseDelay = 100;
bool solenoidActive = false;
bool waitingToRelease = false;
bool button_push = false;

void launcher()
{
	static unsigned long actionStart = 0;

	if (button.start == 1 && !button_push && !solenoidActive)
	{
		// Serial.println("sole active");
		digitalWrite(solenoid, 1);
		actionStart = millis();
		solenoidActive = true;
	}

	// Matikan solenoid setelah 400 ms
	if (solenoidActive && (millis() - actionStart >= pressDuration))
	{
		digitalWrite(solenoid, 0);
		solenoidActive = false;
		waitingToRelease = true;
		actionStart = millis(); // reset waktu untuk delay berikutnya
		button_push = true;
	}

	// Tunggu 100 ms sebelum bisa menekan lagi
	if (waitingToRelease && (millis() - actionStart >= releaseDelay))
	{
		waitingToRelease = false;
	}

	// Reset tombol jika dilepas
	if (button.start == 0)
	{
		button_push = false;
	}
}

unsigned long launcher_motor_prevT = 0;
void launcherRoller(float upper, float lower)
{
	unsigned long launch_currT = micros();
	float deltaT = ((float)(launch_currT - launcher_motor_prevT)) / 1.0e6;

	float launcher_upper_controlled = launcher_up.control_speed(upper, pos[4], deltaT);
	float launcher_lower_controlled = launcher_up.control_speed(lower, pos[5], deltaT);

	esc_first.write(launcher_upper_controlled);
	esc_second.write(launcher_lower_controlled);
	launcher_motor_prevT = launch_currT;
}

joy joySmoothed;
void moveBase()
{
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

	unsigned long currT = micros();
	float deltaT = ((float)(currT - prevT)) / 1.0e6;

	joystick.axis1_x = map(joystick.axis1_x, 0, 128, 0, 2);
	joystick.axis1_y = map(joystick.axis1_y, 0, 128, 0, 2);
	joystick.axis0_y = map(joystick.axis0_y, 0, 128, 0, 2);

	calculate_smooth_vel(joySmoothed.axis1_x, joystick.axis1_x, deltaT, 1.5);
	calculate_smooth_vel(joySmoothed.axis1_y, joystick.axis1_y, deltaT, 1.5);
	calculate_smooth_vel(joySmoothed.axis0_y, joystick.axis0_y, deltaT, 1.5);

	Kinematic::rps req_rps;
	req_rps = kinematic.getRPS(
		joySmoothed.axis1_x,
		joySmoothed.axis1_y,
		joySmoothed.axis0_y,
		-event.orientation.x);

	float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
	float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
	float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);
	float controlled_motor4 = wheel4.control_speed(req_rps.motor4, pos[3], deltaT);

	float current_rps1 = wheel1.get_filt_vel();
	float current_rps2 = wheel2.get_filt_vel();
	float current_rps3 = wheel3.get_filt_vel();
	float current_rps4 = wheel4.get_filt_vel();

	if (fabs(req_rps.motor1) < 0.02)
	{
		controlled_motor1 = 0.0;
	}
	if (fabs(req_rps.motor2) < 0.02)
	{
		controlled_motor2 = 0.0;
	}
	if (fabs(req_rps.motor3) < 0.02)
	{
		controlled_motor3 = 0.0;
	}
	if (fabs(req_rps.motor4) < 0.02)
	{
		controlled_motor4 = 0.0;
	}

	setMotor(cw[0], ccw[0], controlled_motor1);
	setMotor(cw[1], ccw[1], controlled_motor3);
	setMotor(cw[2], ccw[2], controlled_motor2);
	setMotor(cw[3], ccw[3], controlled_motor4);

	Kinematic::velocities vel = kinematic.getVelocities(
		current_rps1,
		current_rps2,
		current_rps3,
		current_rps4);

	unsigned long now = millis();
	float vel_dt = (now - prev_odom_update) / 1000.0;
	prev_odom_update = now;
	odometry.update(
		vel_dt,
		vel.linear_x,
		vel.linear_y,
		vel.angular_z);

	prevT = currT;
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
	if (pwmVal > 0)
	{
		analogWrite(cwPin, fabs(pwmVal));
		analogWrite(ccwPin, 0);
	}
	else if (pwmVal < 0)
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, fabs(pwmVal));
	}
	else
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, 0);
	}
}
template <int j>
void readEncoder()
{
	int b = digitalRead(encb[j]);
	if (b > 0)
	{
		pos[j]++;
	}
	else
	{
		pos[j]--;
	}
}

void calculate_smooth_vel(double &current_output, double input_value, double deltaT, double max_acceleration_)
{
	double target = input_value;

	if (input_value == 0.0 && std::abs(current_output) > 1e-6)
	{
		target = 0.0;
	}
	double max_delta = max_acceleration_ * deltaT;

	double delta_target = target - current_output;

	if (std::abs(delta_target) > max_delta)
	{
		delta_target = (delta_target > 0.0) ? max_delta : -max_delta;
	}

	current_output += delta_target;
}
void button_list(uint32_t buttons, int joy_axis)
{
	/*
	A = 2
	B = 4
	X = 1
	Y = 8
	RB = 20
	Lb = 10
	LT = 40
	RT = 80
	select = 100
	home  = 1000
	start = 200
	*/
	switch (buttons)
	{
	case 2:
		button.A = 1;
		break;

	case 4:
		button.B = 1;
		break;

	case 1:
		button.X = 1;
		break;

	case 8:
		button.Y = 1;
		break;

	case 32:
		button.RB = 1;
		break;

	case 16:
		button.LB = 1;
		break;

	case 64:
		button.LT = 1;
		break;

	case 128:
		button.RT = 1;
		break;

	case 256:
		button.select = 1;
		break;

	case 4096:
		button.home = 1;
		break;

	case 512:
		button.start = 1;
		break;
	default:
		button.A = 0;
		button.B = 0;
		button.X = 0;
		button.Y = 0;
		button.RB = 0;
		button.LB = 0;
		button.LT = 0;
		button.RT = 0;
		button.select = 0;
		button.home = 0;
		button.start = 0;
		break;
	}
	switch (joy_axis)
	{
	case 0:
		button.up = 1;
		break;
	case 2:
		button.right = 1;
		break;
	case 4:
		button.down = 1;
		break;
	case 6:
		button.left = 1;
		break;
	case 15:
		button.up = 0;
		button.down = 0;
		button.left = 0;
		button.right = 0;
		break;
	}
}