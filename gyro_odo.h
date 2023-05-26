#ifndef MANH
#define MAN_H

#include<Arduino.h>
#include "Wire.h"

extern const int ENCR;// YELLOW
extern const int ENCR1; // GREEN
extern const int ENCL; // YELLOW
extern const int ENCL1; //GREEN

extern const int RM; // pwm right motor
extern const int LM; // pwm left motor

// the number of t he Motor Pins
extern const int RM1; //Right motor 1st terminal
extern const int RM2;  //Right motor 2nd terminal
extern const int LM1; //left motor 1st terminal
extern const int LM2;  //left motor 2nd terminal

//Motor Channel
extern const int RM_ch; //PWM channel--timer based
extern const int LM_ch; //PWM channel--timer based

extern const int first_led; //LEDs starts from the Right side and goes to left
extern const int second_led;  
extern const int third_led;
extern const int fourth_led;
extern const int fifth_led; 


// setting PWM properties
// extern const int motors_freq; //Motor Frequency
// extern const int motors_res; //Motor Resoltion 8 bits


//////MOTORS//////

class motori
{
private:
    int encoder;// YELLOW-
    int encoder1; // GREEN-
    int pwm_pin; // pwm right motor-
    int pos; //motor 1st terminal-
    int neg;  //motor 2nd terminal-
    int channel; //PWM channel--timer based-
    int ticks = 0;
    int moving_pwm = 0;
    int all_ticks = 0;

public:
    motori(){}
    motori(int M1,int M2,int M,int M_ch,int ENC,int ENC1);
    bool detect(); //return true after successful calibration
    void tick_increment(){ticks++;}
    void tick_decrement(){ticks--;}
    void setticks(int val){ticks = val;}
    void setmoving_pwm(int val){moving_pwm  = val;}
    void set_all_ticks(int val){ all_ticks += val;}

    void move_forward(int pwm);
    void move_backward(int pwm);
    void donot_move();

    int getticks(){return ticks;}
    int getmoving_pwm(){return moving_pwm;}
    int get_all_ticks(){return all_ticks;}

};


void IRAM_ATTR readEncoderR();
void IRAM_ATTR readEncoderL();



//////GYROSCOPE//////


#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

class MPU6050{
  public:
  MPU6050(){}
  MPU6050(TwoWire &w);
  MPU6050(TwoWire &w, float aC, float gC);

  void begin();

  void writeMPU6050(byte reg, byte data);
  byte readMPU6050(byte reg);

  int16_t getRawAccX(){ return rawAccX; };
  int16_t getRawAccY(){ return rawAccY; };
  int16_t getRawAccZ(){ return rawAccZ; };

  int16_t getRawTemp(){ return rawTemp; };

  int16_t getRawGyroX(){ return rawGyroX; };
  int16_t getRawGyroY(){ return rawGyroY; };
  int16_t getRawGyroZ(){ return rawGyroZ; };

  float getGyroX(){ return gyroX; };
  float getGyroY(){ return gyroY; };
  float getGyroZ(){ return gyroZ; };

	bool calibrateGyro(int tries = 1);

  float getGyroXoffset(){ return gyroXoffset; };
  float getGyroYoffset(){ return gyroYoffset; };
  float getGyroZoffset(){ return gyroZoffset; };

  void update();

  float getAccAngleX(){ return angleAccX; };
  float getAccAngleY(){ return angleAccY; };

  float getGyroAngleX(){ return angleGyroX; };
  float getGyroAngleY(){ return angleGyroY; };
  float getGyroAngleZ(){ return angleGyroZ; };

  float getAngleX(){ return angleX; };
  float getAngleY(){ return angleY; };
  float getAngleZ(){ return angleZ; };



  void setsenstivity(float value){senstivity = value;}
  float getsenstivity(){return senstivity;}

  float bhaluuZangle();

  private:
  void evaluatesensordata();
  float slopempu();

  TwoWire *wire;

  int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
  rawGyroX, rawGyroY, rawGyroZ;

  float gyroXoffset, gyroYoffset;
  float gyroZoffset;

  float temp, accX, accY, accZ, gyroX, gyroY;
  float gyroZ;

  float angleGyroX, angleGyroY;
  float angleGyroZ;
  float angleAccX, angleAccY, angleAccZ;

  float angleX, angleY;
  float angleZ;

  float interval;
  long preInterval;

  float accCoef, gyroCoef;

//shahab's variables for slopes
  float previous = 0;
  unsigned long previoustime = 0;
//getrectified parameters
  float start =0;
  float senstivity = 0.15;
  float angle =0;
};

// class coordinate
// {
// private:
//   int x = 0;
//   int y = 0;
// public:
//   coordinate(int x1, int y1)
//   {
//     x = x1;
//     y = y1;
//   }
//   int get_x(){return x;}
//   int get_y(){return y;}
//   int set_x(int val){x =val;}
//   int set_y(int val){y =val;}
// };

class agent
{
  private:
  
    // coordinate current_cord(0,0);
    // coordinate origin_cord(0,0);
    // coordinate startA_cord(0,0);
    // coordinate goalA_cord(0,0);

    char agent_name = 'S';

    int right_work_lim = 0;
    int left_work_lim = 0;
    // float current_angle = 0.0;
    // float origin_angle = 0;


    void balance_pwm_by_enc(int error2, int scale, char hangel); //motion
    int PDR(float kp, float kd, float ki, int limit, motori parent); //motion
    int PDL(float kp, float kd, float ki, int limit, motori parent); //motion
    bool goal_checkR(int dist_err,int err_lim); //motion
    bool goal_checkL(int dist_err,int err_lim); //motion
    bool deviation_check(int tick_diff,int td_lim);
    void balance_pwm_by_encO(int err, float devit, bool allow, int scale, char hangel);
    int PD2(float kp , float ki, float kd, float angle, motori Parent);
    bool angle_check(float err, float lim);
    char turn_robot(int angle_enc, bool turn_front);

    void forward(int pwml,int pwmr);
    void backward(int pwml,int pwmr);
    void left(int pwml,int pwmr);
    void right(int pwml,int pwmr);
    void brake();

    float my_angle = 0.0;

    //Straight Motion Balancing
    float PDptimeR,PDptimeL,prev_dist_errorR,prev_dist_errorL,ItermR,ItermL;
    int PDprev_errorR , PDprev_errorL;
    bool R_forward = true;
    bool L_forward = true;
    float max_pdR, max_pdL;
    int countR, countL;
    int stuck_countR, stuck_countL;
    int right_pwm, left_pwm;
    int max_diff = 0;
    bool is_stuckedR = false;
    bool is_stuckedL = false;
    int smoothnessL = 0;
    int smoothnessR = 0;

    //PWM Balancing PID
    int prevtick_error_diff = 0;
    double prev_diff_time = 0;
  


    int smoothness = 0;
    double PDc_time2,PDptime2;
    float PDprev_error2 = 0;
    bool PDR_left = true;
    float max_pd2 = 0;
    float prev_angle_error = 0;
    bool is_stucked = false;
    int count_stuck = 15;
    int count = 50;
  public:
    agent(int left_lim, int right_lim, char name);

    void setright_work_lim(int val){right_work_lim = val;}
    void setleft_work_lim(int val){left_work_lim = val;}
    // void setcurrent_angle(float val){current_angle = val;}
    // void setorigin_angle(float val){origin_angle = val;}

    int getright_work_lim(){return right_work_lim;}
    int getleft_work_lim(){return left_work_lim;}
    // int getcurrent_angle(){return current_angle;}
    // int getorigin_angle(){return origin_angle;}


    void setup_motors();
    bool motors_detection();
    bool gyro_detection(int tries = 5);
    char motion(float dist); //moves to the dist cm
    char orientation(float angle);  // changes orientation theta degrees
    char take_turn(int angle);
    void debug_ticks();
};



#endif