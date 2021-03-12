/*******************************************************************************
 FILE_NAME:     desafio.c
 DESCRIPTION:   A controller moving the Pioneer 3-DX and avoiding obstacles.
 DESIGNER:      Juliana Santana
 CREATION_DATE: 08/Mar/2021
 ******************************************************************************/
/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 /* *****************************************************************************
 *        INCLUDES (and DEFINES )
 ******************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/light_sensor.h>



// maximal speed allowed
#define MAX_SPEED 5.24
// how many sensors are on the robot
#define MAX_SENSOR_NUMBER 16
// maximal value returned by the sensors
#define MAX_SENSOR_VALUE 1024
// minimal distance, in meters, for an obstacle to be considered
#define MIN_DISTANCE 1.0
// minimal weight for the robot to turn
#define WHEEL_WEIGHT_THRESHOLD 100
// light limit
#define LIGHT_VALUE 880
/* *****************************************************************************
 *        USER GLOBAL PROCESS VARIABLES
 ******************************************************************************/
// structure to store the data associated to one sensor
typedef struct {
  WbDeviceTag device_tag;
  double wheel_weight[2];
} SensorData;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

// how much each sensor affects the direction of the robot
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

int time_step;
int index_i, index_j;
double speed[2] = {0.0, 0.0};                         
double wheel_weight_total[2] = {0.0, 0.0};
double distance, speed_modifier, sensor_value;
double light_sensor_value_0,light_sensor_value_1,light_sensor_value_2, light_value;
char sensor_name[5] = "";

 WbDeviceTag ls0,ls1,ls2;
 WbDeviceTag left_wheel, right_wheel;
 
 State state;
/* *****************************************************************************
 *        FUNCTIONS AREA
 ******************************************************************************/
static void Init_System(void);             //System initialization
static void Avoid_Obstacle(void);          //Detects obstacles 
static void Stop_Position_Control(void);   //Detects light intensity 
static void Direction_State_Machine(void); //Robot direction setup
static void Set_Motor_Speed(void);         //Robot speed setup  
/* *****************************************************************************
 *                               MAIN FUNCTION
 ******************************************************************************/
int main() {
 // Performs the system initialization //
  Init_System();

   // Main loop //
  while (wb_robot_step(time_step) != -1) {
  
    Avoid_Obstacle();
    Stop_Position_Control();
    Set_Motor_Speed();
  }
  // it closes the communication between the controller and Webots to terminate the controller smoothly
  wb_robot_cleanup();

  return 0;
}

/* *****************************************************************************
 *                             System initialization
 ******************************************************************************/
 static void Init_System(void)
 {
  //initializes the communication between the controller and Webots
  wb_robot_init();

  // stores simulation time step
  time_step = wb_robot_get_basic_time_step();

  // sets up sensors and stores some info about them
  for (index_i = 0; index_i < MAX_SENSOR_NUMBER; ++index_i) {                          
    sprintf(sensor_name, "so%d", index_i);
    sensors[index_i].device_tag = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[index_i].device_tag, time_step);
  }
  
  // stores device IDs for the wheels                                   
  left_wheel = wb_robot_get_device("left wheel");
  right_wheel = wb_robot_get_device("right wheel");
  
  // sets up wheels
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

   // get a handler to the light sensors and enable them
   ls0 = wb_robot_get_device("ls0");
   ls1 = wb_robot_get_device("ls1");
   ls2 = wb_robot_get_device("ls2");
   wb_light_sensor_enable(ls0, time_step);
   wb_light_sensor_enable(ls1, time_step);
   wb_light_sensor_enable(ls2, time_step);

  // by default, the robot goes forward
   state = FORWARD;
 }
 /* *****************************************************************************
 *                             Detects obstacles 
 ******************************************************************************/
 static void Avoid_Obstacle(void)
 {
   // initialize speed and wheel_weight_total arrays at the beginning of the loop
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);

    for (index_i = 0; index_i < MAX_SENSOR_NUMBER; ++index_i) {
      sensor_value = wb_distance_sensor_get_value(sensors[index_i].device_tag);
      // if the sensor doesn't see anything, we don't use it for this round
      if (sensor_value == 0.0)
        speed_modifier = 0.0;
      else {
        // computes the actual distance to the obstacle, given the value returned by the sensor
        distance = 5 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table max value 5.        
        // if the obstacle is close enough, we may want to turn
        // here we compute how much this sensor will influence the direction of the robot
        if (distance < MIN_DISTANCE)
          speed_modifier = 1 - (distance / MIN_DISTANCE);
        else
          speed_modifier = 0.0;
      }
      // add the modifier for both wheels
      for (index_j = 0; index_j < 2; ++index_j) {
        wheel_weight_total[index_j] += sensors[index_i].wheel_weight[index_j] * speed_modifier;
        
        }        
    }
    Direction_State_Machine();
 }
  /* *****************************************************************************
 *                             Detects light intensity 
 ******************************************************************************/
 static void Stop_Position_Control(void)
 {
     // Gets the light sensor value and calculates the average
     light_sensor_value_0 = wb_light_sensor_get_value(ls0);
     light_sensor_value_1 = wb_light_sensor_get_value(ls1);
     light_sensor_value_2 = wb_light_sensor_get_value(ls2);
     light_value = (light_sensor_value_0 + light_sensor_value_1 +light_sensor_value_2)/3;
 }
    /* *****************************************************************************
 *                             Robot speed setup
 ******************************************************************************/
 static void Set_Motor_Speed(void)
 {
     // sets the motor speeds
    wb_motor_set_velocity(left_wheel, speed[0]);
    wb_motor_set_velocity(right_wheel, speed[1]);
    if(light_value > LIGHT_VALUE)// stops next to the floorlight
    {
        wb_motor_set_velocity(left_wheel, 0);
        wb_motor_set_velocity(right_wheel, 0);
    }
 }
     /* *****************************************************************************
 *                             Robot direction setup
 ******************************************************************************/
 static void Direction_State_Machine(void)
 {
  // (very) simplistic state machine to handle the direction of the robot
    switch (state) {
      // when the robot is going forward, it will start turning in either direction when an obstacle is close enough
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.9 * MAX_SPEED;
          speed[1] = -0.9 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.9 * MAX_SPEED;
          speed[1] = 0.9 * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
      // when the robot has started turning, it will go on in the same direction until no more obstacle are in sight
      // this will prevent the robot from being caught in a loop going left, then right, then left, and so on.
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.9 * MAX_SPEED;
          speed[1] = -0.9 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.9 * MAX_SPEED;
          speed[1] = 0.9 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
    }
 }