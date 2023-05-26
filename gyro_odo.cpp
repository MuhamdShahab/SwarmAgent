#include "gyro_odo.h"


motori Right(RM1,RM2,RM,RM_ch,ENCR,ENCR1);
motori Left(LM1,LM2,LM,LM_ch,ENCL,ENCL1);
MPU6050 mpu6050(Wire);

motori::motori(int M1,int M2,int M,int M_ch,int ENC,int ENC1)
{
  encoder = ENC;// YELLOW-
  encoder1 = ENC1; // GREEN-
  pwm_pin = M; // pwm right motor-
  pos = M1; //motor 1st terminal-
  neg = M2;  //motor 2nd terminal-
  channel = M_ch; //PWM channel--timer based-
}

void motori::move_forward(int pwm)
{
    digitalWrite(pos, HIGH);
    digitalWrite(neg, LOW);
    ledcWrite(channel, pwm);
}

void motori::move_backward(int pwm)
{
    digitalWrite(neg, HIGH);
    digitalWrite(pos, LOW);
    ledcWrite(channel, pwm);
}

void motori::donot_move()
{
    digitalWrite(neg, LOW);
    digitalWrite(pos, LOW);
    ledcWrite(channel, 0);
}

bool motori::detect()
{
  for(int i = 8000; i < 35000;i=i+100)
  {
    move_forward(i);
    vTaskDelay(5/portTICK_PERIOD_MS);
    if(abs(ticks) > 10)
    {
      moving_pwm = i+2000;
      donot_move();
      break;
    }
    else
    {
      continue;
    }
  }
  vTaskDelay(100/portTICK_PERIOD_MS);
  donot_move();
  if(moving_pwm == 0)
  {
    ticks = 0;
    Serial.println("  Motor Detection Failed or Check Battery.");
    return false;
  }
  else
  {
    int prevtickdetect = 0;
    int i = moving_pwm;
    int counter = 50;
    while(i > 8000)
    {
      move_forward(i);
      if(prevtickdetect == ticks)
      {
        if(counter <= 0)
        {
          moving_pwm = i;
          donot_move();
          i = 8000;
          break;
        }
        else
        {
          counter--;
        }
      }
      else
      {
        i-=5;
        counter = 50;
      }
      prevtickdetect = ticks;
      vTaskDelay(5/portTICK_PERIOD_MS);
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
    donot_move();
    vTaskDelay(100/portTICK_PERIOD_MS);
    ticks = 0;
    Serial.print("  Motor Detected & Started at PWM value of: ");Serial.println(moving_pwm);
    return true;
  }
}

agent::agent(int left_lim, int right_lim, char name)
{
  right_work_lim = right_lim;
  left_work_lim = left_lim;
  agent_name = name;
}

void agent::setup_motors()
{
  pinMode(ENCR,INPUT_PULLUP);
  pinMode(ENCR1,INPUT_PULLUP);   
  attachInterrupt(digitalPinToInterrupt(ENCR),readEncoderR,RISING);
  pinMode(RM,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  ledcSetup(RM_ch, 30000, 16);
  ledcAttachPin(RM, RM_ch); 
    

  pinMode(ENCL,INPUT_PULLUP);
  pinMode(ENCL1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCL),readEncoderL,RISING);
  pinMode(LM,OUTPUT);
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  ledcSetup(LM_ch, 30000, 16);
  ledcAttachPin(LM, LM_ch);


  pinMode(first_led,OUTPUT);
  pinMode(second_led,OUTPUT);
  pinMode(third_led,OUTPUT);
  pinMode(fourth_led,OUTPUT);
  pinMode(fifth_led,OUTPUT);
  digitalWrite(first_led,LOW);
  digitalWrite(second_led,LOW);
  digitalWrite(third_led,LOW);
  digitalWrite(fourth_led,LOW);
  digitalWrite(fifth_led,LOW);

}

bool agent::motors_detection()
{
 //Starting PWM Values for both motors
  Serial.println("Motors & Gyroscope --> Stay Away Robot will move a little bit.");
  Left.setticks(0);
  Right.setticks(0);
  Serial.print("  1. Left");
  bool val = Left.detect();
  if(val)
  {
    Serial.print("  2. Right");
    val *= Right.detect();
  }
  else
  {
    Serial.println("  Exiting Calibration with error in Left Motor.");
    //val = false;
  }
  if(val)
  {
    val = true;
  }
  else
  {
    Serial.println("  Exiting Calibration with error in Right Motor.");
  }
  Serial.println("========================================");
  return val;
}

int agent::PDR(float kp, float kd, float ki, int limit, motori parent)
{
  int errorR = kp * (limit - abs(parent.getticks()));
  double delta_timeR = (micros() - PDptimeR)/100000;
  PDptimeR = micros();
  ItermR += (ki*(errorR*delta_timeR));
  float edotR = kd * ((errorR-PDprev_errorR)/(delta_timeR));
  float PDR1= (errorR) + (edotR) + (ItermR) ;
  PDprev_errorR = errorR;

  if(PDR1<0)
    R_forward = false;
  else
    R_forward = true;

  if(abs(PDR1) > max_pdR)
  {
    max_pdR = abs(PDR1); //storing the highest number
  }
  else
  {
    //pass
  }
  if(smoothnessR < 15000)
    smoothnessR += 15;
  else
    smoothnessR = 15000;

  return map(constrain(abs(PDR1), 0 , max_pdR), 0,max_pdR,parent.getmoving_pwm(),parent.getmoving_pwm()+smoothnessR);
}

int agent::PDL(float kp, float kd, float ki, int limit, motori parent)
{
  int errorL = kp * (limit - abs(parent.getticks()));
  double delta_timeL = (micros() - PDptimeL)/100000;
  PDptimeL = micros();
  ItermL += (ki*(errorL*delta_timeL));
  float edotL = kd * ((errorL - PDprev_errorL)/(delta_timeL));
  float PDL1= (errorL) + (edotL) + (ItermL) ;
  PDprev_errorL = errorL;
  if(PDL1<0)
    L_forward = false;
  else
    L_forward = true;

  if(abs(PDL1) > max_pdL)
  {
    max_pdL = abs(PDL1); //storing the highest number
  }
  else
  {
    //pass
  }

  if(smoothnessL < 15000)
    smoothnessL += 15;
  else
    smoothnessL = 15000;

  return map(constrain(abs(PDL1), 0 , max_pdL), 0,max_pdL,parent.getmoving_pwm(),parent.getmoving_pwm()+smoothnessL);
}

bool agent::deviation_check(int tick_diff,int td_lim)
{
  if(abs(tick_diff) > abs(td_lim)) //deviation checking and breaking conditions
    return true;
  else
    return false;
}

bool agent::goal_checkR(int dist_err,int err_lim)
{
  if(prev_dist_errorR == dist_err)
  {
    if(dist_err == err_lim)
    {
      if(countR <= 0)
      {
        return true;
      }
      else
      {
        vTaskDelay(2/portTICK_PERIOD_MS);
        countR --;
      }
    }
    else
    {
      if(stuck_countR <= 0)
        is_stuckedR = true;
      else
        stuck_countR--;
    }
  }
  else
  {
    countR = 50;
    stuck_countR = 20;
    is_stuckedR = false;
  }
  prev_dist_errorR = dist_err;
  return false;
}

bool agent::goal_checkL(int dist_err,int err_lim)
{
  if(prev_dist_errorL == dist_err)
  {
    if(dist_err == err_lim)
    {
      if(countL <= 0)
      {
        return true;
      }
      else
      {
        vTaskDelay(2/portTICK_PERIOD_MS);
        countL --;
      }
    }
    else
    {
      if(stuck_countL <= 0)
        is_stuckedL = true;
      else
        stuck_countL--;
    }
  }
  else
  {
    countL = 50;
    stuck_countL = 20;
    is_stuckedL = false;
  }
  prev_dist_errorL = dist_err;
  return false;
}

void agent::balance_pwm_by_encO(int err, float devit, bool allow, int scale, char hangel)
{
  int abs_err = abs(err);
  if( abs_err > max_diff)
    max_diff = abs_err;
  else
  {

  }
  if(err > 0)   
  {
    volatile int offset = 0.00;
    if(abs(devit) <= 1.5 && allow == true)
    {
      offset = map( abs_err+(3*abs(devit)),1,max_diff,50,(max_diff*scale*(1.5+abs(devit))));
    }
    else
    {
      offset = map( abs_err,1,max_diff,50,(max_diff*scale));
    }
    if(hangel == 'M')
    {
      left_pwm = right_pwm + offset;
      right_pwm -=offset;
    }
    else if(hangel == 'M')
    {
      left_pwm = right_pwm - offset;
      right_pwm +=offset;
    }
    else if(hangel == 'O')
    {
      left_pwm += offset;
      right_pwm -= offset;
    }
    else
    {

      //unkown case;
    }
  }
  else if(err  < 0)
  {
    volatile int offset = 0.00;
    if(abs(devit) <= 1.5 && allow == true)
    {
      offset = map( abs_err+(3*abs(devit)),1,max_diff,50,(max_diff*abs(scale)*(1.5+abs(devit))));
    }
    else
    {
      offset = map( abs_err,1,max_diff,50,(max_diff*abs(scale)));
    }
    if(hangel == 'M')
    {
      left_pwm = right_pwm - offset;
      right_pwm += offset;
    }
    else if(hangel == 'M')
    {
      left_pwm = right_pwm + offset;
      right_pwm -=offset;
    }
    else if(hangel == 'O')
    {
      left_pwm -= offset;
      right_pwm += offset;
    }
    else
    {
      //unkown case;
    }
  }
  else
  {
  //do nothing
  }
  right_pwm = constrain(right_pwm, Right.getmoving_pwm(), Right.getmoving_pwm()+20000);
  left_pwm = constrain(left_pwm, Left.getmoving_pwm(), Left.getmoving_pwm()+20000);
        
}

void agent::balance_pwm_by_enc(int error2, int scale, char hangel) // pd2
{


///////the previous version controls the speed by PID if you want to restor the previos version please comment below and uncomment above and comment offset commanf as well

  int offset = 0;
  if(abs(error2) > max_diff)
  {
    max_diff = abs(error2);
  }

  if(error2 > 0)
  {
    offset = map(constrain(abs(error2), 1, max_diff), 1, max_diff, 120, 600* max_diff);
    right_pwm -= offset;
    left_pwm += offset;
  }
  else if(error2 < 0)
  {
    offset = map(constrain(abs(error2), 1, max_diff), 1, max_diff, 120, 600* max_diff);
    right_pwm += offset;
    left_pwm -= offset;
  }
  else
  {}
}

char agent::motion(float dist)
{
  Serial.print("Moving to the Given Distance of ");
  Serial.print(dist);
  Serial.print(" Cms. With tick count of ");
  volatile int limit = ((dist/20.7)*493); //
  Serial.println(limit);

  bool reachedR = false;
  smoothnessR = 0;
  float dist_errR = 0;
  max_diff = 0;
  R_forward = true;
  right_pwm = 0;
  max_pdR = 0;
  countR = 50;
  PDptimeR = 0;
  prev_dist_errorR = 0;
  PDprev_errorR = 0;
  float kpr = 1;
  ItermR = 0;
  is_stuckedR = false;
  Right.setticks(0);
  stuck_countR = 20;
  
  smoothnessL = 0;
  bool reachedL = false;
  float dist_errL = 0;
  left_pwm = 0;
  max_pdL = 0;
  PDptimeL = 0;
  prev_dist_errorL = 0;
  ItermL = 0;
  PDprev_errorL = 0;
  float kpl = 1;
  Left.setticks(0);
  is_stuckedL = false;
  L_forward = true;
  countL = 50;
  stuck_countL = 20;
  bool time_allowed = true;
  int tick_diff = 0;
  double start_time = 0;

  while((!reachedR || !reachedL)) //!reached && !deviated
  {
    //controller design Right
    right_pwm  = PDR(kpr, 0.5, 0, limit, Right);
    dist_errR = ((20.7/493) * (float)(Right.getticks())) - dist;
    Serial.print("  PID#1: DER: "); Serial.print(dist_errR);
    
    if(right_pwm == Right.getmoving_pwm())
      right_pwm = 0;   
    if(is_stuckedR && abs(dist_errR) <= 1.00)
      kpr += 0.3;
    
    //controller design Left
    left_pwm = PDL(kpl, 0.5, 0, limit, Left);
    dist_errL = ((20.7/493) * (float)(Left.getticks())) - abs(dist);
    Serial.print("  DEL: "); Serial.print(dist_errL);

    if(left_pwm == Left.getmoving_pwm())
      left_pwm = 0;
    if(is_stuckedL && abs(dist_errL) <= 1.00)
      kpl += 0.3;


    if((limit - abs(Right.getticks())) <= 5 && (limit - abs(Left.getticks()) ) <= 5 && time_allowed)
    {
      start_time = millis();
      time_allowed = false;
    }

    tick_diff = Right.getticks() - Left.getticks();
    Serial.print("  TD(R-L): ");Serial.println(tick_diff);

    if(left_pwm == 0 || right_pwm == 0)
    {
      // do nothing;
    }
    else
    {
      balance_pwm_by_enc(tick_diff,300,'M');
    }

    //actuator Signal Right
    if(R_forward)
      Right.move_forward(right_pwm);
    else
      Right.move_backward(right_pwm);

    //actuator Signal Left
    if(L_forward)
      Left.move_forward(left_pwm);
    else
      Left.move_backward(left_pwm);

    //Feedback Signal 
    reachedR = goal_checkR(limit - Right.getticks(),0);
    reachedL = goal_checkL(limit - Left.getticks(),0);

    if(time_allowed == false && (millis() - start_time) > 5000)
    {
      brake();
      reachedR = true;
      reachedL = true;
    }
  }

  brake();
  vTaskDelay(250/portTICK_PERIOD_MS);
  Left.set_all_ticks(Left.getticks());
  Right.set_all_ticks(Right.getticks());
  Left.setticks(0);
  Right.setticks(0);
  // Serial.print("  All Time Tick for Left are: ");Serial.println(Left.get_all_ticks());
  // Serial.print("  All Time Tick for Right are: ");Serial.println(Right.get_all_ticks());
  if(reachedR && reachedL)
  {
    Serial.println("  Success: Desired Location Reached. Func returned Char S");
    Serial.println("================Success================");
    return 'S';
  }
  else
  {
    Serial.println("  Failure: Could'nt Reach the Desired Distance or unknown error. Func returned Char E");
    Serial.println("================Failure================");
    return 'E';
  }
}

char agent::turn_robot(int angle_enc, bool turn_front)
{
  bool right_on = true;
  bool pre_def = false;
  if(angle_enc == 45 || angle_enc == 90)
  { 
    right_on = true;
    pre_def = true;
  }
  else if(angle_enc == -45 || angle_enc == -90)
  {
    right_on = false;
    pre_def = true;
  }
  else
  {
    pre_def = false;
  }
  if(pre_def)
  {
    angle_enc = abs(angle_enc);
    Serial.print("Taking turn for the angle of ");
    Serial.print(angle_enc);
    Serial.println(" Degrees.");

    int limit = (float)angle_enc * 7.2674;
    
    Left.setticks(0);
    Right.setticks(0);
    motori parent = Right;
    bool reached_angle = false;
    int parent_pwm = 0;
    float kpp = 1;
    float angle_err = 0.0;
    is_stuckedR = false;
    R_forward = false;

    max_pdR = 0;
    countR = 50;
    PDptimeR = 0;
    prev_dist_errorR = 0;
    PDprev_errorR = 0;
    ItermR = 0;
    is_stuckedR = false;
    Right.setticks(0);
    Left.setticks(0);   
    stuck_countR = 20;
    double start_time = 0;
    bool time_allowed = true;
    while(!reached_angle)
    {
      if(right_on)
      {
        parent = Right; 
      }
      else
      {
        parent = Left;
      }

      //controller design Right
      parent_pwm  = PDR(kpp, 0.5, 0, limit, parent);
      int er_enc = limit - abs(parent.getticks());
      Serial.print("  #2PD: TED: "); Serial.println(er_enc);
      if(parent_pwm == Right.getmoving_pwm() && right_on)
        parent_pwm = 0;
      else if(parent_pwm == Left.getmoving_pwm() && !right_on)
        parent_pwm = 0;
      else
      {
        //pass
      }
      if(is_stuckedR)
      {
        kpp += 0.3;
        if(time_allowed && abs(er_enc) < 5)
          {
            start_time = millis();
            time_allowed = false;
          }
      }
      if(turn_front)
      {
      if(R_forward)
        parent.move_forward(parent_pwm);
      else
        parent.move_backward(parent_pwm);
      }
      else
      {
      if(R_forward)
        parent.move_backward(parent_pwm);
      else
        parent.move_forward(parent_pwm);
      }
      reached_angle = goal_checkR(er_enc,0);

      if(millis() - start_time > 5000 && !time_allowed)
        reached_angle = true;
    }
    brake();
    vTaskDelay(250/portTICK_PERIOD_MS);
    Serial.print("Final Tick Error: "); Serial.println(limit - abs(parent.getticks()));
    if(reached_angle)
    {
      Serial.println("  Success: Desired Location Reached. Func returned Char S");
      Serial.println("================Success================");
      return 'S';
    }
    else
    {
      Serial.println("  Time Exceeded: Could'nt reach in Given time. Func returned Char W");
      Serial.println("================Warning================");
      return 'W';
    }
  }
  else
  {
    Serial.println("  Error: Angle Out of Range. Func returned Char E");
    Serial.println("================Error================");
    return 'E';
  }
}

char agent::take_turn(int angle)
{
  if(angle == 45)
  {
    if(gyro_detection())
    turn_robot(angle,true);
    else
    {
      Serial.print("Gyro Not Found");
    }
  }
  else if(angle == -45)
  {
    if(gyro_detection())
    turn_robot(angle,true);
    else
    {
      Serial.print("Gyro Not Found");
    }
  }
  else if(angle == 90)
  {
    if(gyro_detection())
    turn_robot(angle,true);
    else
    {
      Serial.print("Gyro Not Found");
    }
  }
  else if(angle == -90)
  {
    if(gyro_detection())
    turn_robot(angle,true);
    else
    {
      Serial.print("Gyro Not Found");
    }
  }
  else if(angle == 135)
  {

  }
  else if(angle == -135)
  {

  }
  else if(angle == 180 || angle == -180 )
  {
    turn_robot(90,true);
    turn_robot(-90,false);
    motion(7.3);
  }
  else if(0)
  {

    return 'S';
  }
  else
  {
    Serial.println(" Unknown Angle!!!");
    return 'O';
  }
}
// Motors Directions with PWM
void agent::forward(int pwml,int pwmr)
{
  Left.move_forward(pwml);
  Right.move_forward(pwmr);
}

void agent::debug_ticks()
{
  while(1){
  Serial.print("Left Motor: ");
  Serial.print(Left.getticks());
  Serial.print("  Right Motor: ");
  Serial.println(Right.getticks());
  }
}

void agent::backward(int pwml,int pwmr)
{
  Left.move_backward(pwml);
  Right.move_backward(pwmr);
}

void agent::left(int pwml,int pwmr)
{
  Left.move_backward(pwml);
  Right.move_forward(pwmr); 

}

void agent::right(int pwml,int pwmr)
{
  Left.move_forward(pwml);
  Right.move_backward(pwmr);
}

void agent::brake()
{
  Right.donot_move();
  Left.donot_move();
}

//Interrupt Functions for left and right motors
void IRAM_ATTR readEncoderR()
{
  if(digitalRead(ENCR1) > 0)
    Right.tick_increment();
  else
    Right.tick_decrement();
}
void IRAM_ATTR readEncoderL()
{
  if(digitalRead(ENCL1) > 0)
    Left.tick_increment();
  else
    Left.tick_decrement();
}



///////////
///////////
//GYROSCOPE
///////////
///////////


MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  return wire->read();
}

void MPU6050::evaluatesensordata()
{
  volatile float prevangle = bhaluuZangle();
  vTaskDelay(300/ portTICK_PERIOD_MS);
  update();
  if(bhaluuZangle() != prevangle)
  {
    Serial.println("  Issues Found! Repeating Process. Suggestion: Keep Gyroscope still.");
    Wire.begin();
    begin();
    bool val = calibrateGyro();
  }
  else
  {
    Serial.println("  Gyroscope Detected.");
    Serial.println("========================================");
    start = getAngleZ();
    previoustime = millis();
    previous =  getAngleZ();
  }
}

bool MPU6050::calibrateGyro(int tries)
{
  start = 0;
  previous = 0;
  angle = 0;
  previoustime = 0;
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;
	if(tries > 4){
    Serial.print("Detecting Gyroscope --> ");
    Serial.println("DO NOT MOVE MPU6050");
  }
  for(int i = 0; i < 500; i++)
  { 
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x43);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 6);

    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 65.5;
    y += ((float)ry) / 65.5;
    z += ((float)rz) / 65.5;
  }
  gyroXoffset = x / 500;
  gyroYoffset = y / 500;
  gyroZoffset = z / 500;
  bool val = 1;
  if(gyroXoffset == gyroYoffset && gyroYoffset == gyroZoffset)
  {
    Serial.print("  Attempts Left# ");
    Serial.print(tries);
    for(int i  = 0; i < 5;i++)
    {
      Serial.print(" .");
      vTaskDelay(50/ portTICK_PERIOD_MS);
    }
    Serial.println();
    tries --;
    if(tries > 0)
    {
      Wire.begin();
      begin();
      val = calibrateGyro(tries);
    }
    else
    {
      Serial.println("  Error: MPU6050 Not Found!");
      Serial.println("========================================");
      return 0;
    }
  }
  else
  {
    evaluatesensordata();
    return val;
  }
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read();
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;

  angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;
 
  gyroX = ((float)rawGyroX) / 65.5;
  gyroY = ((float)rawGyroY) / 65.5;
  gyroZ = ((float)rawGyroZ) / 65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ *interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis();
}

float MPU6050::slopempu()
{
  update();
  if((millis() - previoustime) >= 20)
  {
    volatile int slope = (((getAngleZ() - previous)*1000)/(millis() - previoustime));
    previoustime = millis();
    previous = getAngleZ();
    return abs(slope);
  }
  else
  {
    return 0;
  }
}
//part of above statements
float MPU6050::bhaluuZangle()
{
  volatile int diff = getAngleZ()- start;
  if(slopempu() > senstivity)
  {
    angle = getAngleZ() - diff;
    start = getAngleZ();
  } 
  return angle;
}

bool agent::gyro_detection(int tries)
{
  Wire.begin();
  mpu6050.begin();
  return mpu6050.calibrateGyro(tries);
}

int agent::PD2(float kp2, float ki2, float kd2, float error2, motori Parent2)
{
  PDc_time2= micros();
  double delta_time2 = (PDc_time2 - PDptime2)/100000.0;
  PDptime2 = PDc_time2;
  volatile float edot2 = (error2-PDprev_error2)/delta_time2;
  PDprev_error2 = error2;
  float eint2 = (error2*delta_time2);

  volatile float PD2= (kp2*error2) + (kd2*edot2) + (ki2*eint2);
  // Serial.print("  P: ");Serial.print(kp2*error2);
  // Serial.print("  I: ");Serial.print(ki2*eint2);
  // Serial.print("  D: ");Serial.print(kd2*edot2);
  // Serial.print("  PID: ");Serial.print(PD2);


  if(PD2 > 0) // if angle is positive or left turn is required
    PDR_left = true;
  else
    PDR_left = false;
  if(abs(PD2) > max_pd2)
  {
    max_pd2 = abs(PD2); //storing the highest number
  }
  else
  {
    //pass
  }

  if(smoothness <= 15000)
    smoothness += 20;
  else
    smoothness = 15000;

  return map(constrain(abs(PD2), 0 , max_pd2), 0.0,max_pd2,Parent2.getmoving_pwm(),Parent2.getmoving_pwm()+smoothness);
}

bool agent::angle_check(float angle_err,float err_lim)
{
  if(prev_angle_error == angle_err)
  {
    if(abs(angle_err) < abs(err_lim))
    {
      if(count <= 0)
      {
        return true;
      }
      else
      {
        count --;
      }
    }
    else
    {
      if(count_stuck <= 0)
        is_stucked = true;
      else
        count_stuck--;
    }
  }
  else
  {
    count = 30;
    count_stuck = 7;
    is_stucked = false;
  }
  prev_angle_error = angle_err;
  return false;
}

char agent::orientation(float angle)
{
  Serial.print("Taking turn for the given angle ");
  Serial.print(angle);
  Serial.println(" Degrees.");
  Jump:
  float start_angle_turn = mpu6050.bhaluuZangle();
  for(int i = 0; i < 5 ;i++)
  {
    start_angle_turn = mpu6050.bhaluuZangle();
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
  float anglel = mpu6050.bhaluuZangle();
  for(int i = 0; i < 5 ;i++)
  {
    anglel = mpu6050.bhaluuZangle();
    vTaskDelay(50/portTICK_PERIOD_MS);
  }
  if(anglel != start_angle_turn)
  {
    Serial.println("  Gyro Issue found at start of Orientation Changing. Applying Fixes.");
    gyro_detection(2);
    goto Jump;
  }
  else
  {
    start_angle_turn = mpu6050.bhaluuZangle();
  }
  PDc_time2 = 0,
  PDptime2 = 0;
  PDprev_error2 = 0;
  PDR_left = true;
  max_pd2 = 0;
  prev_angle_error = 0;
  smoothness = 0;
  right_pwm = 0;
  left_pwm = 0;
  bool turned = false;
  count = 30;
  prev_angle_error = 0;
  float ki = 0.9;
  float kp = 1.20;
  float kd = 1.9;
  is_stucked = false;
  count_stuck = 7;
  Left.setticks(0);
  Right.setticks(0);
  PDR_left = true;
  float err = 0;
  int tick_diff = 0;
  bool time_exceeded = false;
  bool fluctuated = false;
  int count_fluc = 4;
  bool prev_PDR_Left = PDR_left;
  my_angle = mpu6050.bhaluuZangle();
  while(!turned)
  {
    err =(angle + start_angle_turn) - my_angle;
    tick_diff = abs(Right.getticks()) - abs(Left.getticks());
    my_angle = mpu6050.bhaluuZangle();
    Serial.print("  PID#2: AE: "); Serial.print(err);
    Serial.print("  TD(R-L): "); Serial.println(tick_diff);
    

    // Controller

    if(abs(err) > 1.0 && is_stucked)
    {
      kp +=0.3;
    }
    else if(abs(err) <= 1.0 && !is_stucked)
    {
      kp = 1.20;
    }
    else
    {

    }
    my_angle = mpu6050.bhaluuZangle();
    if(abs(err) <= 1.0 && is_stucked)
    {
      ki += 3;
    }
    else
    {

    }
    my_angle = mpu6050.bhaluuZangle();
    right_pwm = PD2(kp,ki, kd, err,Right);
    left_pwm = map(right_pwm,Right.getmoving_pwm(),Right.getmoving_pwm()+smoothness,Left.getmoving_pwm(),Left.getmoving_pwm()+smoothness);
    if(right_pwm == Right.getmoving_pwm())
    {
      my_angle = mpu6050.bhaluuZangle();
      right_pwm = 0;
      left_pwm = 0;
    }
    else
    {
      my_angle = mpu6050.bhaluuZangle();
      balance_pwm_by_encO(tick_diff,0,false,800, 'O');
    }
    
    // Signal to Actuator
    if(prev_PDR_Left != PDR_left)
    { 
      brake();
      vTaskDelay(50/portTICK_PERIOD_MS);
      brake();
      vTaskDelay(25/portTICK_PERIOD_MS);
      brake();
      Serial.println("  Brakedddd");
    }
    else if(PDR_left)                  
      left(left_pwm, right_pwm);
    else
      right(left_pwm, right_pwm);
    my_angle = mpu6050.bhaluuZangle();
    prev_PDR_Left = PDR_left;
    vTaskDelay(5/portTICK_PERIOD_MS);
    my_angle = mpu6050.bhaluuZangle();
    turned = angle_check(err,0.50); 
  }
  brake();
  vTaskDelay(250/portTICK_PERIOD_MS);
  Left.setticks(0);
  Right.setticks(0);
  if(turned)
  {
    Serial.println("  Orientation Completed. Func returned Char T");
    Serial.println("================Success================");
    return 'T';
  }
  else
  {
    Serial.println("  Could'nt Reach the Desired Distance or unknown error. Func returned Char E");
    Serial.println("================Failure================");
    return 'E';
  }
}