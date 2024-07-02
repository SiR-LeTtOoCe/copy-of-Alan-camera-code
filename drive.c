#include "drive.h"
#include <math.h>
//#include <kipr/botball.h>
#include <kipr/wombat.h>
#include <stdio.h>
#include <stdlib.h>

double GYRO_DEV;
double function_start;

int start_time(){
    return seconds();
}

void slow_servo(int port, int pos, int time){

    int diff = get_servo_position(port) - pos;
    int inc = 80;
    //printf("dif is %d \n", diff);
    int i = 0;	
    while(i < inc){
        set_servo_position(port, get_servo_position(port) - diff/inc);
        msleep(time/inc);
        i++;            
    }
    set_servo_position(port, pos);
    msleep(200);
}

void drive_straight_hs(int speed, float counter) {
    float left_speed = speed;													
    float right_speed = speed;												
    double offset = 0;			
    // clear motor position counters							
    clear_motor_position_counter(MOT_LEFT);						
    clear_motor_position_counter(MOT_RIGHT);	
    drive(left_speed*1.2, right_speed);
    msleep(200);
    double prev_tme = seconds();
    // keep driving while the condition is not fulfilled (1) and timeout not triggered
    while(abs(gmpc(MOT_LEFT)) < counter) {	
        // find the gyro value based on wallaby orientation
        double val = 0;		
        int i;
        for (i = 0; i < 10; ++i) val += (gyro_z() + 6);
        val /= 10;
        // keep a running offset of the gyrcoo value to know how far off the robot is
        offset += val * (seconds() - prev_tme);
        //printf("val: %f \n", val);
        //printf("offset %f \n", offset);
        prev_tme = seconds();
        // recalculate speeds based on how far off the robot is from the drive path
        left_speed = speed - ((float){offset} * HIGH_GYRO_SENS);
        right_speed = speed + ((float){offset} * HIGH_GYRO_SENS);
        //printf("seconds(): %f, function_start: %f\n", seconds(), function_start);
        drive(left_speed, right_speed);
    }
    stop();
}


void drive_straight(int speed, float counter) {
    float left_speed = speed;													
    float right_speed = speed;												
    double offset = 0;			
    // clear motor position counters							
    clear_motor_position_counter(MOT_LEFT);						
    clear_motor_position_counter(MOT_RIGHT);	
    double prev_tme = seconds();
    // keep driving while the condition is not fulfilled (1) and timeout not triggered
    while(abs(gmpc(MOT_LEFT)) < counter) {	
        // find the gyro value based on wallaby orientation
        double val = 0;
        val = (gyro_z() - GYRO_DEV);
        // val /= 2;
        // keep a running offset of the gyro value to know how far off the robot is
        offset += val * (seconds() - prev_tme);
        // printf("val: %f \n", val);
        // printf("offset %f \n", offset);
        prev_tme = seconds();
        // recalculate speeds based on how far off the robot is from the drive path
        left_speed = speed - ((float){offset} * GYRO_SENS);
        right_speed = speed + ((float){offset} * GYRO_SENS);
        drive(left_speed, right_speed);
        msleep(15);
    }
    stop();
}

void drive_straight_nc(int speed, float counter) {
    float left_speed = speed;													
    float right_speed = speed;												
    double offset = 0;			
    // clear motor position counters							
    double prev_tme = seconds();
    // keep driving while the condition is not fulfilled (1) and timeout not triggered
    drive(speed * 1.5, speed);
    msleep(200);
    while(abs(gmpc(MOT_LEFT)) < counter) {	
        // find the gyro value based on wallaby orientation
        double val = 0;		
        int i;
        for (i = 0; i < 10; ++i) val += (gyro_z());
        val /= 10;
        // keep a running offset of the gyrcoo value to know how far off the robot is
        offset += val * (seconds() - prev_tme);
        //printf("val: %f \n", val);
        //printf("offset %f \n", offset);
        prev_tme = seconds();
        // recalculate speeds based on how far off the robot is from the drive path
        left_speed = speed - ((float){offset} * HIGH_GYRO_SENS);
        right_speed = speed + ((float){offset} * HIGH_GYRO_SENS);
        //printf("seconds(): %f, function_start: %f\n", seconds(), function_start);
        drive(left_speed, right_speed);
        msleep(50);
    }
    stop();
    msleep(100);
}

void line_follow_left(int port, int speed, int distance){ 
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(abs(gmpc(MOT_LEFT)) < distance) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        //printf("%f\n", frac);
        int l_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT;
        int r_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}

void line_follow_left_until_anal(int port, int port2, int speed) { 
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(analog(port2) < 2000) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        //printf("%f\n", frac);
        int l_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT;
        int r_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}

void line_follow_left_highsens(int port, int speed, int distance){ 
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(abs(gmpc(MOT_LEFT)) < distance) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        //printf("%f\n", frac);
        int l_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        int r_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}

void line_follow_left_highsens_until_anal(int port, int port2, int speed, int line_val)
{
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(analog(port2) < line_val) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        //printf("%f\n", frac);
        int l_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        int r_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}

void line_follow_right_highsens(int port, int speed, int distance){ 
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(abs(gmpc(MOT_LEFT)) < distance) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        //printf("%f\n", frac);
        int l_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        int r_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_LEFT_HIGH;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}
/*
void square_up(int dir, int pulses){
    int speed = dir * 1000;
    int i = 0;
    for(i = 0; i < pulses; i++){
        drive(speed, 0);
        msleep(200);
        drive(0, speed);
        msleep(200);
    }
    drive(speed, speed);
    msleep(300);
    stop();
}
*/
void twerk (int strokes){
    int i = 0;
    drive(300, -300);
    msleep(100);
    for(i = 0; i < strokes; i++){
        drive(-300, 300);
        msleep(200);
        drive(300, -300);
        msleep(200);
        stop();
    }
    drive(-300, 300);
    msleep(100);
    stop();
}


void line_follow_right(int port, int speed, int distance){ 
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(abs(gmpc(MOT_LEFT)) < distance) {  
        double frac = (analog(port) - WHITE_VAL) / (BLACK_VAL - WHITE_VAL);
        int l_speed = speed + (frac - 0.5) * speed * LINE_FOLLOW_SENS_RIGHT;
        int r_speed = speed - (frac - 0.5) * speed * LINE_FOLLOW_SENS_RIGHT;
        //printf("%f %d %d \n", frac, l_speed, r_speed);
        drive(l_speed, r_speed);  
        msleep(5);
        //console_clear();
    }
    drive(0, 0);
}

void digital_line_follow(int port, int speed, int distance, float sens){
    cmpc(MOT_LEFT);
    cmpc(MOT_RIGHT);
    while(abs(gmpc(MOT_LEFT)) < distance) {  
        if(analog(port) < GRAY_VAL){
            drive(speed + speed*sens, speed - speed*sens);
        }
        else{
            drive(speed - speed*sens, speed + speed*sens);
        }
        msleep(10);
    }
    drive(0, 0);
}

/*
void turn(int l_speed, int r_speed, double deg) {
    // remember start of function for timeout
    int current_gyro_ticks = 0;
    double final_gyro_ticks = deg * GYRO_PER_ROT / 360.0;
    int prev_tme = seconds();
    // while not enough turning done and timeout of 20 seconds
    while(abs(current_gyro_ticks) < abs(final_gyro_ticks)) {
        drive(l_speed, r_speed);
        msleep(20);
        // keep track of a runnning gyro offset
        // determine gyro value based on wallaby orientation
        double val = 0;
        int i;
        for (i = 0; i < 10; ++i) val += (int) {(gyro_z()) / 3} * 3;
        val /= 10;
        current_gyro_ticks += val * (seconds() - prev_tme);
        prev_tme = seconds();
        printf("%d\n", current_gyro_ticks);
    }
    // stop motors to prevent residual momentum
    // wait a bit before next movement to prevent residual momentum from affecting other gyro functions
    stop();
}
*/

void turn(int speed, double deg) {			
    speed *= deg / abs(deg);
    // clear motor position counters							
    clear_motor_position_counter(MOT_LEFT);						
    clear_motor_position_counter(MOT_RIGHT);	
    // keep driving while the condition is not fulfilled (1) and timeout not triggered
    double counter = abs(deg) * 10;
    while(abs(gmpc(MOT_LEFT)) < counter) {	
        drive(speed, -speed);
        msleep(15);
    }
    stop();
}

void drive_straight_until_anal(int port, int anal_val, int speed) {
    cmpc(MOT_LEFT);
    float left_speed = speed;													
    float right_speed = speed;												
    double offset = 0;			
    // clear motor position counters							
    double prev_tme = seconds();
    // keep driving while the condition is not fulfilled (1) and timeout not triggered
    if(anal_val > 0){
        while(analog(port) < anal_val){
            double val = 0;		
            int i;
            for (i = 0; i < 10; ++i) val += gyro_z();
            val /= 10;
            // keep a running offset of the gyrcoo value to know how far off the robot is
            offset += val * (seconds() - prev_tme)- .0;
            //printf("val: %f \n", val);
            //printf("offset %f \n", offset);
            prev_tme = seconds();
            // recalculate speeds based on how far off the robot is from the drive path
            left_speed = speed - ((float){offset} * GYRO_SENS);
            right_speed = speed + ((float){offset} * GYRO_SENS);
            //printf("seconds(): %f, function_start: %f\n", seconds(), function_start);
            drive(left_speed, right_speed);
            //msleep(100);
        }
        stop();
    }
    else{
        while(analog(port) > abs(anal_val)){
            double val = 0;		
            int i;
            for (i = 0; i < 10; ++i) val += gyro_z();
            val /= 10;
            // keep a running offset of the gyrcoo value to know how far off the robot is
            offset += val * (seconds() - prev_tme);
            //printf("val: %f \n", val);
            //printf("offset %f \n", offset);
            prev_tme = seconds();
            // recalculate speeds based on how far off the robot is from the drive path
            left_speed = speed - ((float){offset} * GYRO_SENS);
            right_speed = speed + ((float){offset} * GYRO_SENS);
            //printf("seconds(): %f, function_start: %f\n", seconds(), function_start);
            drive(left_speed, right_speed);
            //msleep(100);
        }
        stop();
    }
	printf("%d\n", gmpc(MOT_LEFT));
    stop();
}

void turn_until_analog(int port, int val, int l_speed, int r_speed){
    if(val > 0){
        while(analog(port) < val){
            drive(l_speed, r_speed);
        }
    }
    else{
        while(analog(port) > abs(val)){
            drive(l_speed, r_speed);
        }
    }
    drive(0,0);
    msleep(10);
}

void turn_until_gyro(int l_speed, int r_speed, double deg, int slow_down) {
    // remember start of function for timeout
    function_start = seconds();
    double current_gyro_ticks = 0;
    double final_gyro_ticks = deg * GYRO_PER_ROT / 360.0;
    double prev_tme = seconds();
    // while not enough turning done and timeout of 20 seconds
    while(abs(current_gyro_ticks) < abs(final_gyro_ticks) && seconds() - 4 < function_start) {
        if (slow_down == 1 && abs(final_gyro_ticks) - abs(current_gyro_ticks) < (abs(l_speed) + abs(r_speed)) / 2.3) 
            drive(l_speed > 0 ? 60 : -60, r_speed > 0 ? 60 : -60);
        else
            drive(l_speed, r_speed);
        // keep track of a runnning gyro offset
        // determine gyro value based on wallaby orientation
        double val = 0;
        //int i;
        // for (i = 0; i < 10; ++i) 
        val = gyro_z() - GYRO_DEV;
        //val /= 10;
        current_gyro_ticks += val * (seconds() - prev_tme);
        prev_tme = seconds();
        //printf("%f\n", current_gyro_ticks);
        msleep(15);
        //console_clear();
    }
    // stop motors to prevent residual momentum
    drive(0, 0);
    // wait a bit before next movement to prevent residual momentum from affecting other gyro functions
    msleep(50);
}

void drive(int l_speed, int r_speed){
    mav(MOT_LEFT, l_speed);
    mav(MOT_RIGHT, r_speed);

}

void mtp2(int port, int speed, int goal_pos){
    cmpc(port);
    if(gmpc(port) > goal_pos){
        speed = abs(speed) * -1;
    }

    while(abs(abs(gmpc(port)) - abs(goal_pos)) > 5){
        mav(port, speed);
    }
    /*
    while(abs(gmpc(port)) != abs(goal_pos)){
        mav(port, 100 *speed / abs(speed));
    }
    */
    drive(0,0);
    msleep(50);
}

void wrist_reset(int mot_port, int toggle){ //toggle: 1 or -1
    mav(mot_port, toggle * 1000); //used to be 1500
    msleep(100);
    int ticks = 0;
    do{
        ticks = gmpc(mot_port); 
        mav(mot_port, toggle * 500); //used to be 1000
        msleep(50);

        //printf("%d\n", gmpc(mot_port) - ticks);
    } while(abs(gmpc(mot_port) - ticks) > 20);
    //printf("done\n");
    drive(0,0);
    msleep(1000);
    off(mot_port);
    cmpc(mot_port);
    msleep(100);
}


void low_key(){
    void key_one_thread(){
        slow_servo(SERVO_SHOULDER, 832, 1000);
    }

    msleep(1000);
    set_servo_position(SERVO_CLAW, 496);
    slow_servo(SERVO_SHOULDER, 1601, 1000);
    slow_servo(SERVO_ELBOW, 450, 1000);
    msleep(500);

    thread key_one;
    key_one = thread_create(&key_one_thread);
    thread_start(key_one);
    msleep(300);

    //slow_servo(SERVO_SHOULDER, 832, 1000);
    slow_servo(SERVO_ELBOW, 1472, 600);
    msleep(1000);
}

void mid_key(){
    void key_two_thread(){
        slow_servo(SERVO_SHOULDER, 838, 1000);
    }

    msleep(1000);
    set_servo_position(SERVO_CLAW, 146);
    slow_servo(SERVO_SHOULDER, 1566, 1000);
    slow_servo(SERVO_ELBOW, 300, 1000);
    msleep(500);

    thread key_two;
    key_two = thread_create(&key_two_thread);
    thread_start(key_two);
    msleep(300);

    //slow_servo(SERVO_SHOULDER, 832, 1000);
    slow_servo(SERVO_ELBOW, 1250, 600);
}

void high_key(){
    void key_three_thread(){
        slow_servo(SERVO_SHOULDER, 1100, 1000);
    }
    msleep(1000);
    set_servo_position(SERVO_CLAW, 765);
    slow_servo(SERVO_SHOULDER, 1540, 1000);
    slow_servo(SERVO_ELBOW, 630, 500);
    msleep(500);
    slow_servo(SERVO_ELBOW, 376, 1000);
    msleep(500);
    set_servo_position(SERVO_CLAW, 777);
    thread key_three;
    key_three = thread_create(&key_three_thread);
    thread_start(key_three);
    msleep(400);


    slow_servo(SERVO_ELBOW, 900, 750);
    msleep(1000);
    set_servo_position(SERVO_CLAW, 420);
}

void stop(){
    drive(0,0);
    msleep(10);
}

void calc_dev() {
    printf("please keep robot still for 7 seconds\n press r_button when ready\n");
    while (!right_button()) {
        msleep(50);
        //printf("%f \n", gyro_z());
    }
    printf("calculating...\n");
    int time = 5000;
    int interval = 50;
    double sum = 0;
    double i;
    for (i = 0; i < time / interval; ++i) {
        double val = gyro_z();
        sum += val;
        msleep(interval);
    }
    GYRO_DEV = sum / i;
    printf("average deviation of %f \n", GYRO_DEV);
}


int wait_for_light_new() {
    int off_val;
    int on_val;
    while (1) {
        while (!right_button()) {
            display_clear();
            printf("Please turn the light on.\nPress right button when ready.\nCurrent value of %d seen\n", analog(LIGHT_START_PORT));
            msleep(30);
        }
        while (right_button()) msleep(30);
        int sm = 0;
        int i = 0;
        for (i = 0; i < 20; ++i) {
            display_clear();
            printf("Calibrating.... Current value of %d.\n", analog(LIGHT_START_PORT));
            sm += analog(LIGHT_START_PORT);
            msleep(50);
        }
        on_val = sm/20;
        while (!right_button()) {
            display_clear();
            printf("On value of %d\n", on_val);
            printf("Please turn the light off.\nPress right button when ready.\nCurrent value of %d seen\n", analog(LIGHT_START_PORT));
            msleep(30);
        }
        while (right_button()) msleep(30);
        on_val = sm / i;
        sm = 0;
        for (i = 0; i < 20; ++i) {
            display_clear();
            printf("Calibrating.... Current value of %d.\n", analog(LIGHT_START_PORT));
            sm += analog(LIGHT_START_PORT);
            msleep(50);
        }
        off_val = sm / i;
        display_clear();
        printf("Off value of %d\nOn value of %d.\n", off_val, on_val);
        if (on_val > 700 || off_val < 2500 || abs(on_val - off_val) < 2000) {
            printf("Bad calibration! Press right button to redo, A button to override\n");
            while (!a_button() && !right_button()) msleep(50);
            if (right_button()) {
                while (right_button()) msleep(50);
                printf("Redoing calibration!\n");
                msleep(1500);
            } else {
                while (a_button()) msleep(50);
                printf("Overriding the calibration!\n");
                msleep(2000);
                break;
            }
        } else {
            printf("Good calibration!\n");
            msleep(2000);
            break;
        }
    }

    while(1){
        printf("Currently reading %d\n", analog(LIGHT_START_PORT));
        if (analog(LIGHT_START_PORT) < (double) {off_val - on_val} * 0.3 + on_val) {
            printf("Found a value exceeding %f\n", (double) {off_val - on_val} * 0.3 + on_val);
            msleep(10);
            return 0;
        } 
        console_clear();
    }
}

void hardware_check(){
    drive_straight(1000, 300);
    drive_straight(-1000, 300);
	turn_until_gyro(1000, -1000, 90, 0);
    stop();
    msleep(500);
}


void start_up() {
    enable_servos();
    //slow_servo(SERVO_ELBOW, ELBOW_START, 500);
    //slow_servo(SERVO_SHOULDER, SHOULDER_START, 500);
    //set_servo_position(SERVO_CLAW, CLAW_DOWN);
    //set_servo_position(SERVO_COUPLER, COUPLER_START);
    msleep(100);
    calc_dev();
    msleep(200);
}
