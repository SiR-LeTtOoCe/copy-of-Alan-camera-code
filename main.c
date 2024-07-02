#include <kipr/wombat.h>
#include "drive.h"

// change move_arm method

const int CLAW_CLOSED = 230;
const int CLAW_OPEN = 600;

const int MID_X = 45;
const int MID_Y = 80;

const int DUMPER_UP = 850;
const int DUMPER_DOWN = 1600;

void raise_dumper()
{
    set_servo_position(1, DUMPER_UP);
}

void lower_dumper()
{
    set_servo_position(1, DUMPER_DOWN);
}


void open_claw()
{
    set_servo_position(0, CLAW_OPEN);
}

void close_claw()
{
    set_servo_position(0, CLAW_CLOSED);
}

void drive_ticks(int speed, int ticks)
{
    drive(speed, speed);
    int start = gmpc(MOT_LEFT);
    while(abs(gmpc(MOT_LEFT) - start) < ticks)
    {
        msleep(10);
    }
    drive(0, 0);
}

void turn_ticks(int left_speed, int right_speed, int ticks)
{
    drive(left_speed, right_speed);
    int start = gmpc(MOT_LEFT);
    while(abs(gmpc(MOT_LEFT) - start) < ticks)
    {
        msleep(10);
    }
    drive(0, 0);
}

void move_arm_to_top()
{
    mav(2, 1000);
    msleep(4300);
    mav(2, 0);
}

void move_arm(int speed, int ticks)
{
    int start = gmpc(2);
    mav(2, speed); // change 1000 to speed
    while(abs(gmpc(2) - start) < ticks)
    {
        msleep(10);
    }
    mav(2, 0);
}

void zero_arm(int dir)
{
    mav(3, 300 * dir);
    int curr;
    do
    {
        curr = gmpc(3);
        msleep(100);
    } while(abs(curr - gmpc(3)) > 1);
    mav(3, -150 * dir);
    msleep(500);
    mav(3, 0);
}

void turn_arm_ticks(int dir, int ticks)
{
    int curr = gmpc(3);
    mav(3, 300 * dir);
    while(abs(curr - gmpc(3)) < ticks)
    {
		msleep(10);
    }
	mav(3, 0);
}

void init()
{
    enable_servo(0);
    enable_servo(1);
    
    raise_dumper();
    open_claw();
    move_arm_to_top();

    zero_arm(1);

    camera_load_config("new");
    camera_open_at_res(LOW_RES);
}

void shake_pom()
{
    turn_arm_ticks(1, 20);
    turn_arm_ticks(-1, 20);
    turn_arm_ticks(1, 20);
    turn_arm_ticks(-1, 20);
    turn_arm_ticks(1, 20);
    turn_arm_ticks(-1, 20);
}

void light_blue_pom(int dir)
{
    printf("light blue\n");
    if(dir == 1)
        turn_arm_ticks(-1, 50);
    else
        turn_arm_ticks(-1, 300);
    
    move_arm(-1000, 600);
    
    int count = 0;
    while(count < 3)
    {
        msleep(1000);
        
        camera_update();
        int area = get_object_area(0, 0);
        if(area < 100)
        {
            continue;
        }

        camera_update();
        point2 center = get_object_centroid(1, 0);
        int y = center.x;
        int delta_y = -(y - MID_Y);    
        printf("Delta y: %d\n", delta_y);


        if(delta_y > 0)
        {
            turn_arm_ticks(-1, delta_y);
        }
        else
        {
            turn_arm_ticks(1, -delta_y);
        }

        camera_update();
        center = get_object_centroid(1, 0);
        int x = center.y;
        int delta_x = (x - MID_X);
        printf("Delta x: %d\n", delta_x);
        if(delta_x < 0)
        {
            drive_ticks(200, -delta_x * 8);
            count++;
        }
        else
        {
            drive_ticks(-200, delta_x * 8);
            count++;
        }
    }
    
    close_claw();
    msleep(500);
    move_arm(-1000, 990);
    msleep(500);
    set_servo_position(0, 370);
    msleep(500);
    move_arm(-1000, 810);
    msleep(500);
    set_servo_position(0, 400);
    msleep(500);
    close_claw();
    msleep(500);
    move_arm_to_top();
    
    turn_arm_ticks(-dir, 200);
    msleep(500);
    open_claw();
    msleep(500);
    zero_arm(1);
}

void grab_dark_blue_poms(int dir)
{
    if(dir == 1)
        turn_arm_ticks(-1, 50); // 1, 100
    else
        turn_arm_ticks(-1, 300); // -1, 300
    
    move_arm(-1000, 600);
    
    int count = 0;
    while(count < 3)
    {
        msleep(1000);
        
        camera_update();
        int area = get_object_area(0, 0);
        if(area < 100)
        {
            continue;
        }

        camera_update();
        point2 center = get_object_centroid(0, 0);
        int y = center.x;
        int delta_y = -(y - MID_Y);    
        printf("Delta y: %d\n", delta_y);


        if(delta_y > 0)
        {
            turn_arm_ticks(-1, delta_y);
        }
        else
        {
            turn_arm_ticks(1, -delta_y);
        }

        camera_update();
        center = get_object_centroid(0, 0);
        int x = center.y;
        int delta_x = (x - MID_X);
        printf("Delta x: %d\n", delta_x);
        if(delta_x < 0)
        {
            drive_ticks(200, -delta_x * 8);
            count++;
        }
        else
        {
            drive_ticks(-200, delta_x * 8);
            count++;
        }
    }
    
    camera_update();
    printf("%d %d\n", get_object_centroid(0, 0).y, get_object_centroid(0, 0).x);

    close_claw();
    msleep(500);
    move_arm(-1000, 990);
    msleep(500);
    set_servo_position(0, 370);
    msleep(500);
    move_arm(-1000, 810);
    msleep(500);
    set_servo_position(0, 400);
    msleep(500);
    close_claw();
    msleep(500);
    move_arm_to_top(); 
    
    shake_pom();
    
    zero_arm(-1);
    msleep(500);
    open_claw();
	msleep(500);
    zero_arm(1);

    msleep(2000);

    open_claw();
}

void grab_two_poms(int dir)
{
    zero_arm(1);
    msleep(500);

    int i;
    for(i = 0; i < 2; i++)
    {   
        light_blue_pom(dir);
        grab_dark_blue_poms(dir);

        drive_straight(1000, 800);
    }
}

void square_up(int dir) // 1 = left corner, -1 = right corner (closer to solor panel)
{
    if(dir == 1)
		drive_straight(1000, 2000);
    else
        drive_straight(1000, 3500);
    
    
    msleep(100);
    drive_straight(-1000, 300);
    msleep(100);
    turn_until_gyro(-750 * dir, 750 * dir, 90, 0);
    msleep(100);
    drive_straight(1000, 2000);
    msleep(100);
    drive_straight(-1000, 600);
}

void move_sides()
{
    drive_straight(-1000, 300);
    turn_until_gyro(750, -750, 90, 0);
    drive_straight(-1000, 2200);
    turn_until_gyro(-750, 750, 90, 0);
}

void demo_astronaut()
{
    move_arm(-1000, 2500);
    msleep(500);
    close_claw();
    msleep(500);
    move_arm_to_top();
    msleep(500);
    zero_arm(-1);
    msleep(500);
    open_claw();
    msleep(500);
    zero_arm(1);
}

int main()
{
    init();
    
    printf("\nWaiting for button\n");
    while(!right_button())
    {
        msleep(10);
    }
    
    //demo_astronaut();
    
    start_up();
    
    //turn_until_gyro(500, -500, 90, 0);
    
    printf("\nWaiting for button\n");
    while(!right_button())
    {
        msleep(10);
    }
    
    int start_time = seconds();
    
    square_up(1);
    
    grab_two_poms(1);
    
    move_sides();
    
    square_up(-1);
    
    grab_two_poms(-1);
    
    lower_dumper();
    
    camera_close();
    
    printf("Time: %f\n", seconds() - start_time);
    
    return 0;
}
