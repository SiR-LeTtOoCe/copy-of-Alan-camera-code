//#include <kipr/botball.h>

// adjust GYRO based on the wallaby orientation
//   -1, 1 for gyro_x()
//   -2, 2 for gyro_y()
//   -3, 3 for gyro_z()

#define GYRO 3
#define GYRO_SENS 20
#define HIGH_GYRO_SENS 10.
#define GYRO_PER_ROT 2830
//#define GYRO_PER_ROT 355

#define FRONT_LIGHT_PORT 0
#define BACK_LIGHT_PORT 1
#define LIGHT_START_PORT 2
#define LINE_FOLLOW_SENS_RIGHT 1
#define LINE_FOLLOW_SENS_LEFT .6

#define LINE_FOLLOW_SENS_LEFT_HIGH .5

#define WHITE_VAL 300.0
#define BLACK_VAL 3000.0
#define GRAY_VAL 1000.0

#define MOT_LEFT 1
#define MOT_RIGHT 0

#define SERVO_SHOULDER 2
#define SERVO_CLAW 3
#define SERVO_ELBOW 0
#define SERVO_COUPLER 1

#define SHOULDER_START 1100
#define SHOULDER_UPRIGHT 1300
#define SHOULDER_BACK 850
#define SHOULDER_FIRST 1050//1250
#define SHOULDER_SECOND 1050//1050
#define SH_KEY_ONE 1400
//1300
#define COUPLER_OPEN 0
#define COUPLER_CLOSED 800
#define COUPLER_CLAMPED 1100
#define COUPLER_START 1750

#define CLAW_DOWN 1935

#define ELBOW_START 100
#define ELBOW_RESET 400
#define ELBOW_FIRST 1100
#define ELBOW_SECOND_ONE 1200
#define ELBOW_SECOND_TWO 1000
#define ELBOW_FLOOR 1440
#define EL_KEY_ONE 700


void wrist_reset(int mot_port, int toggle);
void slow_servo();
void drive(int l_speed, int r_speed);
void drive_straight(int speed, float counter);
void drive_straight_hs(int speed, float counter);
void drive_straight_until_anal(int port, int anal_val, int speed);
void drive_straight_nc(int speed, float counter);
void line_follow_left(int port, int speed, int distance);
void line_follow_left_highsens(int port, int speed, int distance);
void line_follow_left_until_anal(int port, int port2, int speed);
void line_follow_left_highsens_until_anal(int port, int port2, int speed, int line_val);
void line_follow_right(int port, int speed, int distance);
void line_follow_right_highsens(int port, int speed, int distance);
void digital_line_follow(int port, int speed, int distance, float sens);
//void turn(int l_speed, int r_speed, double deg);
void turn(int speed, double deg);
void drive_until_analog(int port, int val, int speed);
void turn_until_analog(int port, int val, int l_speed, int r_speed);
void turn_until_gyro(int l_speed, int r_speed, double deg, int slow_down);
void mtp2(int port, int speed, int goal_pos);
// void square_up(int dir, int pulses);
void twerk (int strokes);
void low_key();
void mid_key();
void high_key();
void start_up();
int wait_for_light_new();
void calc_dev();
void hardware_check();
void stop();
