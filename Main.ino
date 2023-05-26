#include <Astar_swarm.h>
#include <gyro_odo.h>
#include <ArduinoJson.h>


int start_x = 0;
int start_y = 0;
int data_start = 0;
int data_end = 1;

int current_x = 0;
int current_y = 0;

//int start_x = 4;
//int start_y = 0;
//int data_start = 1;
//int data_end = 3;

//int start_x = 8;
//int start_y = 0;
//int data_start = 3;
//int data_end = 4;

// Core definitions (assuming you have dual-core ESP32)
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

const int COL = (360 / 30);
const int ROW = (120 / 30);

const int ENCR = 32;// YELLOW-
const int ENCR1 = 33; // GREEN-
const int ENCL = 34; // YELLOW-
const int ENCL1 = 35; //GREEN-

const int RM = 13; // pwm right motor-
const int LM = 25; // pwm left motor-

// the number of the Motor Pins
const int LM1 = 26; //Left motor 1st terminal-
const int LM2 = 27;  //Left motor 2nd terminal-
const int RM1 = 12; //Right motor 1st terminal-
const int RM2 = 14;  //Right motor 2nd terminal-

//Motor Channel
const int RM_ch = 0; //PWM channel--timer based-
const int LM_ch = 1; //PWM channel--timer based-

const int first_led = 15; //Left motor 1st terminal-
const int second_led = 2;  //Left motor 2nd terminal-
const int third_led = 0; //Right motor 1st terminal-
const int fourth_led = 4;  //Right motor 2nd terminal-
const int fifth_led = 5; //Left motor 1st terminal-

agent Agent(0, 0, 'B');

#define RXD2 16
#define TXD2 17

int gx[5] = {2, 3, 5, 7, 11};
int gy[5] = {2, 1, 3, 2, 1};

int goal_x = 3;
int goal_y = 3;
int left_lim = 0;
int right_lim = 12;
int current_angle = 0;
int required_angle = 0;
float onset = 0.0;
bool onces = true;



int** my_obstacles(int** arr)
{
  arr = place_obstacle(arr, 1, 1);
  arr = place_obstacle(arr, 6, 1);
  arr = place_obstacle(arr, 10, 2);
  return arr;
}

void next_coord()
{
  int i = 0;
  while (i < 20)
  {
    digitalWrite(second_led, HIGH);
    digitalWrite(fourth_led, HIGH);
    delay(300);
    digitalWrite(second_led, LOW);
    digitalWrite(fourth_led, LOW);
    delay(300);
    i++;
  }
}

void start_coord(bool cond)
{
  if(cond == true)
  {
   digitalWrite(first_led, HIGH); 
   digitalWrite(third_led, HIGH); 
   digitalWrite(fifth_led, HIGH); 
  }
  else
  {
   digitalWrite(first_led, LOW); 
   digitalWrite(third_led, LOW); 
   digitalWrite(fifth_led, LOW); 
  } 
  
}

void goal_coord()
{
  start_coord(false);
  vTaskDelay(100/portTICK_PERIOD_MS);
  digitalWrite(first_led, HIGH);
  int i = 0;
  while (i < 20)
  {
    delay(150);
    digitalWrite(second_led, HIGH);
    delay(150);
    digitalWrite(third_led, HIGH);
    delay(150);
    digitalWrite(fourth_led, HIGH);
    delay(150);
    digitalWrite(fifth_led, HIGH);
    delay(500);
    digitalWrite(fifth_led, LOW);
    delay(150);
    digitalWrite(fourth_led, LOW);
    delay(150);
    digitalWrite(third_led, LOW);
    delay(150);
    digitalWrite(second_led, LOW);
    i++;
  }
  digitalWrite(first_led, LOW);
  vTaskDelay(100/portTICK_PERIOD_MS);
  start_coord(true);
}

String robotPackage = "1";

void serial_send()
{
  DynamicJsonDocument doc(512);
  JsonArray jsonArray = doc.to<JsonArray>();
  jsonArray.add(current_x);
  jsonArray.add(current_y);

  String jsonString;
  serializeJson(doc, jsonString);
  jsonString = robotPackage + jsonString;

  // Sending data to bot via Port 2
  char sendData[(jsonString.length() + 1)];
  jsonString.toCharArray(sendData, jsonString.length() + 1);
  Serial.println(sizeof(sendData));
  Serial.println(sendData);
  Serial2.write(sendData);
}

int path_size = 0;
const int num_stops = 15;
int arr_angle[num_stops] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float arr_dist[num_stops] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float arr_off[num_stops+1] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int cord_x[num_stops] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int cord_y[num_stops] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void refresh_array()
{
  for(int i = 0; i < num_stops; i++)
  {
    arr_angle[i] = 0;
    arr_dist[i] = 0;
    arr_off[i] = 0;
  }
}

float getonset(int angle)
{
  if (angle == 45 || angle == -45)
    return 3.0238;
  else if (angle == 90 || angle == -90)
    return 7.3;
  else if (angle == 0 || angle == -0)
    return 0;
  else
  {
    Serial.println(" Got unknown thing in getonset");
    return 0;
  }
}

void A_star()
{
  int ** arena =  getmap(1, COL, ROW); //generate first param map like 1.
  arena = my_obstacles(arena);//place your obstacles
  astar_result apath = Astar(start_x, start_y, goal_x, goal_y, arena); //gives a 2D array of feasible path with number of coordinates
  Serial.println("  A-star found the following path: "); printmap(apath.getpath(), apath.getsize(), 2); //prints the coordinates with getsize number of columns and 2(x and y cordinates)
  Serial.print("  We have to traverse the following number of coordinates: "); Serial.println(apath.getsize());
  path_size = apath.getsize()-1;
  if (apath.getsuccess() == true)
  {
    int temp = 0;
    float demp = 0;
    float femp = 0;
    for (int i = 0; i <= path_size; i++)
    {
      Serial.print("  Astar: ");
      if (((apath.getpath()[0][i] == goal_x) && (apath.getpath()[1][i] == goal_y)) ||i == path_size)
      {
        Serial.print("Current Angle of the Robot at goal is "); Serial.print(current_angle); Serial.print(" Degrees from the starting Orientation i.e 0 ");
        Serial.println("Goal Reached by Robot !!!!!!!!");
        temp = 0;
        femp = temp;
        demp = femp;
      }
      else
      {
        if (apath.getpath()[0][i] == apath.getpath()[0][i + 1] && apath.getpath()[1][i + 1] == (apath.getpath()[1][i] + 1)) // F
        {
          required_angle = 0;
          temp = required_angle - current_angle;
          current_angle = required_angle;
          onset = getonset(temp);    
          demp = 30 - onset - 7.3;   
          Serial.print("Take Turn for F "); Serial.print(temp);
          Serial.print(" and move distance of "); Serial.println(demp);
          femp = 7.3 - onset;
          if (femp < 2)    
            femp = 0;
        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] + 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i] + 1))) // FL
        {
          required_angle = 45;
          temp = required_angle - current_angle;
          current_angle = required_angle;
          onset = getonset(temp);
          demp = 42.2 - onset - 7.3;   
          Serial.print("Take Turn for FL "); Serial.print(temp);
          Serial.print(" and move distance of "); Serial.println(demp);
          femp = 7.3 - onset;
          if (femp < 2)    
            femp = 0;
        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] - 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i] + 1))) // FR
        {
          required_angle = -45;
          temp = required_angle - current_angle;
          current_angle = required_angle;
          onset = getonset(temp);
          demp = 42.2 - onset - 7.3;   
          Serial.print("Take Turn for FR "); Serial.print(temp);
          Serial.print(" and move distance of "); Serial.println(demp);
          femp = 7.3 - onset;
          if (femp < 2)    
            femp = 0;
        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] - 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i]))) // L
        {
          required_angle = -90;
          int temp = required_angle - current_angle;
          current_angle = required_angle;
          onset = getonset(temp);
          demp = 30 - onset - 7.3;   
          Serial.print("Take Turn for L "); Serial.print(temp);
          Serial.print(" and move distance of "); Serial.println(demp);
          femp = 7.3 - onset;
          if (femp < 2)    
            femp = 0;
        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] + 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i]))) // R
        {
          required_angle = 90;
          temp = required_angle - current_angle;
          current_angle = required_angle;
          onset = getonset(temp);
          demp = 30 - onset - 7.3;   
          Serial.print("Take Turn for R "); Serial.print(temp);    
          Serial.print(" and move distance of "); Serial.println(demp);
          femp = 7.3 - onset;
          if (femp < 2)    
            femp = 0;
        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] + 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i] - 1))) // BL //NC
        {

        }
        else if ((apath.getpath()[0][i] == apath.getpath()[0][i + 1]) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i] - 1))) // B
        {

        }
        else if ((apath.getpath()[0][i + 1] == apath.getpath()[0][i] - 1) && (apath.getpath()[1][i + 1] == (apath.getpath()[1][i] - 1))) // BR //NC
        {
          
        }
        else
        {
          Serial.println("Boom !!! Unknown error");
        }
      }
      if(i == path_size - 1)
        demp += 7.3;
      arr_off[i] = femp;
      arr_angle[i] = temp;
      arr_dist[i] = demp;
      cord_x[i] = apath.getpath()[0][i];
      cord_y[i] = apath.getpath()[1][i];
    }
  }
  else
  {
    refresh_array();
    Serial.println("No Path Found");
  }
}

void move_goal()
{
  Serial.println("Moving to Goals: ");
  int temp = 0;
  float demp = 0.0;
  float femp = arr_off[0];
  Serial.print("  ");
  Serial.print("X");
  Serial.print("  ");
  Serial.print("Y");
  Serial.print("  ");
  Serial.print("Angle");
  Serial.print("  ");
  Serial.print("Distance");
  Serial.print("  ");
  Serial.println("Offset");
    
  if(abs(femp) > 2)  
    Agent.motion(femp);
  for(int i = 0; i <= path_size ; i++)
  {
    temp = arr_angle[i];
    demp = arr_dist[i];
    femp = arr_off[i+1];
    current_x = cord_x[i];
    current_y = cord_y[i];
    Serial.print("  ");
    Serial.print(current_x);
    Serial.print("  ");
    Serial.print(current_y);
    Serial.print("  ");
    Serial.print(temp);
    Serial.print("  ");
    Serial.print(demp);
    Serial.print("  ");
    Serial.println(femp);

    vTaskDelay(500 / portTICK_PERIOD_MS); 
    //serial_send();
    if(i < path_size)    
      next_coord();
    else
    {
      goal_coord();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if(abs(temp) >= 45)
      Agent.take_turn(temp);

    if(abs(demp + femp) > 2)
      Agent.motion(demp + femp);
  }
}
void retrieve_back()
{
  Serial.println("Retrieving Starting pt: ");
  Serial.println("Turning 180 Degrees");
  Agent.take_turn(180);
   int temp = 0;
  float demp = 0.0;
  float femp = 0;
  Serial.print("  ");
  Serial.print("X");
  Serial.print("  ");
  Serial.print("Y");
  Serial.print("  ");
  Serial.print("Angle");
  Serial.print("  ");
  Serial.print("Distance");
  Serial.print("  ");
  Serial.println("Offset");
    
  for(int i = path_size; i >= 0 ; i--)
  {
    temp = arr_angle[i];
    demp = arr_dist[i];
    femp = arr_off[i+1];
    current_x = cord_x[i];
    current_y = cord_y[i];
    Serial.print("  ");
    Serial.print(current_x);
    Serial.print("  ");
    Serial.print(current_y);
    Serial.print("  ");
    Serial.print(temp);
    Serial.print("  ");
    Serial.print(demp);
    Serial.print("  ");
    Serial.println(femp);

    vTaskDelay(500 / portTICK_PERIOD_MS); 
    //serial_send();

    vTaskDelay(500 / portTICK_PERIOD_MS);
    if(abs(demp + femp) > 2)
      Agent.motion(demp + femp);  
    next_coord();
    if(abs(temp) >= 45)
      Agent.take_turn(-temp);
}
  Agent.motion(7.3);
  Agent.take_turn(180);
  refresh_array();
  
}

void find_coords()
{
  if (onces)
  {
    for (int i = data_start; i < data_end ; i++)
    {
      delay(5000);
      goal_x = gx[i];
      goal_y = gy[i];
      A_star();
    }
    onces = false;
  }
  else
  {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    Serial.println("Waiting for the New Goal point in FindCOORDs.");
  }
}

void tito(String jsonString)
{
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, jsonString);

  // Retrieve the JSON array from the parsed JSON object
  JsonArray jsonArray = doc.as<JsonArray>();
  data_start = jsonArray[0].as<int>();
  data_end = jsonArray[1].as<int>();
  Serial.println(data_start);
  Serial.println(data_end);
}




char swarmPackage = '1';

unsigned long curr = millis();

void serial_rec()
{
  if(millis()-curr >= 20000) {
    Serial.println("20s past");
    curr = millis();
  }
  if (Serial2.available() >= 1) {
    char number = Serial2.read();
    if (number == 'q') {
      Serial.println("q");
      number = Serial2.read();
      if (number == 'w') {
        Serial.println(number);
        Serial.println("Data Received...");
        String incoming = Serial2.readStringUntil('\n');
        if (incoming[0] == swarmPackage)
        {
          //jsonString = incoming;
          incoming = incoming.substring(1);
          tito(incoming);
          find_coords();
        }
        else
          incoming = "";
      }
      else
      {
        //Serial.println("No data Available");
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //Serial.println("================ ALPHA ================");

//  Serial.println("Serial Cummunication Enabled");
//  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
//  Serial.println("Let's Start");

  Agent.setup_motors();
  bool gyr = Agent.gyro_detection();
  bool motr = Agent.motors_detection();
}

void loop()
{
  //serial_rec();
  find_coords();
  start_coord(true);
  vTaskDelay(1000/portTICK_PERIOD_MS);
  move_goal();
  vTaskDelay(5000/portTICK_PERIOD_MS); // will return to start after this
  retrieve_back();
  start_coord(false);
  vTaskDelay(10000/portTICK_PERIOD_MS); //align to start manually
  while(1)
  {
    
  }
}
