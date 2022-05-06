#define MOTOR1_A 2
#define MOTOR1_B 3  // ШИМ!
#define MOTOR2_A 4
#define MOTOR2_B 5  // ШИМ!
#define MOTOR3_A 7
#define MOTOR3_B 6  // ШИМ!
#define MOTOR4_A 8
#define MOTOR4_B 9  // ШИМ!

#include "math.h"

#include <GyverMotor.h>
// тут можно поменять моторы местами
GMotor motorFL(DRIVER2WIRE, MOTOR1_A, MOTOR1_B, HIGH);
GMotor motorBL(DRIVER2WIRE, MOTOR2_A, MOTOR2_B, HIGH);
GMotor motorFR(DRIVER2WIRE, MOTOR3_A, MOTOR3_B, HIGH);
GMotor motorBR(DRIVER2WIRE, MOTOR4_A, MOTOR4_B, HIGH); 

//GMotor motorBL(DRIVER2WIRE, MOTOR1_A, MOTOR1_B, HIGH);
//GMotor motorFL(DRIVER2WIRE, MOTOR2_A, MOTOR2_B, HIGH);
//GMotor motorBR(DRIVER2WIRE, MOTOR3_A, MOTOR3_B, HIGH);
//GMotor motorFR(DRIVER2WIRE, MOTOR4_A, MOTOR4_B, HIGH); 

int movement_speed = 30;

// пины энкодеров
#define OPTO_FL 13
#define OPTO_FR 12
#define OPTO_BL 10
#define OPTO_BR 11
#include "encoder.h"  // мини-класс для щелевого датчика
encCounter encFL(OPTO_FL);
encCounter encFR(OPTO_FR);
encCounter encBL(OPTO_BL);
encCounter encBR(OPTO_BR);

bool write_counter = false;
bool debug = false;
bool checking = false;

class speedKeeper {
public:
  encCounter *ec;
  GMotor *motor;

  // motor static parameters
  int friction = 20;
  float power_scale = 1;
  int difference_fix = 5;
  int direction = 1;
  int tick_length = 0.01;

  // dynamic variables
  int speed = 0;  // current speed
  int speed_next = 0; // calculated speed
  int ticks = 0;  // ticks from last update
  long pos = 0;   // current wheel position
  // history
  int speed_history[10];
  long tick_previous = 0;
  
  // control variables
  int gain = 0;
  int target_spd_prev = 0;
  int target_speed = 0;
  int target_speed_prev = 0;
  long target_pos = 0;
  int pos_difference = 0;

  // self-handling variables
  bool controlled = true;
             int mtimer = 10; // timer between engiene updates
  long ms = 0;      // time in the last update
  
  speedKeeper(encCounter *ec, GMotor *motor) {
    this->ec = ec;
    this->motor = motor;
    ms = millis();
  }

  void update() {
    long cticks = ec->update(1);
    if (tick_previous != 0 && cticks > tick_previous) {
      ticks += cticks - tick_previous;
    }
    
    tick_previous = cticks;
  }

  // interface method
  void keep(int target_spd) {
    if (target_spd != target_spd_prev) {
      target_spd_prev = target_spd;
      target_speed_prev = 0;
      //pos = target_pos - difference_fix;
    }
    if (target_spd == 0) {
      motor->setMode(BRAKE);
    }
    if (target_spd * direction < 0) {
      motor->setMode(BACKWARD);
    }
    if (target_spd * direction > 0) {
      motor->setMode(FORWARD);
    }
    
    target_speed = abs(target_spd);
    
    if (!controlled) {
      update();
      if (millis() - ms > mtimer) {
        makestep(millis() - ms);
        ms = millis();
      }
    }
  }

  // tick calculating
  void tick(int mtimer) {
      speed = ticks * 1000 / mtimer;
      pos += ticks;
      target_pos += target_speed * mtimer / 1000;
      target_pos = min(pos + 100, target_pos);
      pos_difference = target_pos - pos;

      speed_register(speed);
      ticks = 0;
  }

  //calculate and send motor value
  void motor_handle() {
    int speed_difference = max(0, target_speed - speed);
    //Serial.println(pos_difference);
    gain = max((pos_difference /*+ difference_fix*/ + speed_difference * 0.25) * 0.7, 0) * 5;
    motor->setSpeed(gain * power_scale);
  }

  // make full tick with motor handle
  void makestep(int mtimer) {
    tick(mtimer);
    motor_handle();
  }

  // calculate next frame speed
  int pos_predict(int ms) {
    return speed * ms;
  }
  
  // send speed value to the history array
  void speed_register(int speed) {
    for(int i = 9; i >= 1; i--) {
      speed_history[i] = speed_history[i - 1];
    }
    speed_history[0] = speed;
  }
};
speedKeeper speedFR(&encFR, &motorFR);
speedKeeper speedFL(&encFL, &motorFL);
speedKeeper speedBR(&encBR, &motorBR);
speedKeeper speedBL(&encBL, &motorBL);

class QuadController {
public:
  speedKeeper* speed0;
  speedKeeper* speed1;
  speedKeeper* speed2;
  speedKeeper* speed3;
  long ms;
  int mtimer = 60;
  
  QuadController(speedKeeper* s0, speedKeeper* s1, speedKeeper* s2, speedKeeper* s3) {
    speed0 = s0;
    speed1 = s1;
    speed2 = s2;
    speed3 = s3;
    speed0->controlled = true;
    speed1->controlled = true;
    speed2->controlled = true;
    speed3->controlled = true;
  }

  void tick() {

    speed0->update();
    speed1->update();
    speed2->update();
    speed3->update();
    
    if (millis() - ms > mtimer) {
      int timediff = millis() - ms;
      ms = millis();

      speed0->tick(timediff);
      speed1->tick(timediff);
      speed2->tick(timediff);
      speed3->tick(timediff);

      int max_difference = max(max(speed0->pos_difference, speed1->pos_difference), max(speed2->pos_difference, speed3->pos_difference));
      //Serial.println(max_difference - speed0->pos_difference);
      //Serial.println(max_difference - speed1->pos_difference);
      //Serial.println(max_difference - speed2->pos_difference);
      //Serial.println(max_difference - speed3->pos_difference);
      
      //speed0->target_pos -= (max_difference - speed0->pos_difference) * 0.01;
      //speed1->target_pos -= (max_difference - speed1->pos_difference) * 0.01;
      //speed2->target_pos -= (max_difference - speed2->pos_difference) * 0.01;
      //speed3->target_pos -= (max_difference - speed3->pos_difference) * 0.01;

      if (debug) {
        Serial.println(speed0->pos_difference);
        Serial.println(speed1->pos_difference);
        Serial.println(speed2->pos_difference);
        Serial.println(speed3->pos_difference);
        Serial.println("max_difference: " + String(max_difference));
      }
      
      speed0->motor_handle();
      speed1->motor_handle();
      speed2->motor_handle();
      speed3->motor_handle();
    }
  }
};

QuadController quad_controller(&speedFL, &speedFR, &speedBL, &speedBR);

void UP(){
  speedFL.keep(movement_speed);
  speedFR.keep(movement_speed);
  speedBL.keep(movement_speed);
  speedBR.keep(movement_speed);
}

void DOWN(){
  speedFL.keep(-movement_speed);
  speedFR.keep(-movement_speed);
  speedBL.keep(-movement_speed);
  speedBR.keep(-movement_speed);
}
void RIGHT(){
  speedFL.keep(-movement_speed);
  speedFR.keep(movement_speed);
  speedBL.keep(movement_speed);
  speedBR.keep(-movement_speed);
  
}
void LEFT(){       
  speedFL.keep(movement_speed);
  speedFR.keep(-movement_speed);
  speedBL.keep(-movement_speed);
  speedBR.keep(movement_speed);
}

void STAY(){       
  speedFL.keep(0);
  speedFR.keep(0);
  speedBL.keep(0);
  speedBR.keep(0);
}

void motorcheck() {
  
  Serial.println("front left");
  motorFL.setMode(AUTO);
  motorFL.setSpeed(100);
  delay(3000);
  motorFL.setMode(STOP);
  
  Serial.println("front right");
  motorFR.setMode(AUTO);
  motorFR.setSpeed(100);
  delay(3000);
  motorFR.setMode(STOP);
  
  Serial.println("back left");
  motorBL.setMode(AUTO);
  motorBL.setSpeed(100);
  delay(3000);
  motorBL.setMode(STOP);
  
  Serial.println("back right");
  motorBR.setMode(AUTO);
  motorBR.setSpeed(100);
  delay(3000);
  motorBR.setMode(STOP);
}

void setup() {
  
  Serial.begin(9600);
  
  if (checking)
    motorcheck();
  
  quad_controller.speed2->power_scale = 3;
  quad_controller.speed2->difference_fix = 7;
  
  quad_controller.speed3->direction = -1;
  quad_controller.speed3->power_scale = 0.9;
  quad_controller.speed3->difference_fix = 7;

  quad_controller.speed1->difference_fix = 10;
  
  quad_controller.speed0->difference_fix = 7;
  quad_controller.speed0->direction = -1;
}

bool scenario_enable = false;
int scenario_index = -1;
const int scenario_count = 8;
String scenario[scenario_count] = {"left","stop", "up","stop", "right","stop", "down","stop",};
int scenario_step_ms[scenario_count] = {1500, 500, 1500, 500, 1500, 500, 1500, 500};
int scenario_pause = 100;
int scenario_next_timer = 0;
int scenario_ms = millis();

int state = 0;
String command_buffer;
void loop() {
  quad_controller.tick();

  // process commands
  String command = "";
  if (Serial.available()) {
    command_buffer += (char)(Serial.read());
    for(int i = 0; i < command_buffer.length(); i++) {
      char s = command_buffer[i];
      if (s == '\n') {
        command = command_buffer.substring(0, i);
        command_buffer = command_buffer.substring(i + 1, command_buffer.length());
        break;
      }
    }
  }

  if (command.length()) {
    Serial.println("Recieved command: `" + command + "`");
  }

  // check if scenario is executing
  if (scenario_enable && command.length() == 0) {
    scenario_next_timer -= millis() - scenario_ms;
    scenario_ms = millis();
    if (scenario_next_timer < 0) {
      scenario_index = (scenario_index + 1) % scenario_count;
      scenario_next_timer = scenario_step_ms[scenario_index];
      command = scenario[scenario_index];
    }
  }

  if (state == 0) {STAY();}
  if (state == 1) {RIGHT();}
  if (state == 2) {DOWN();}
  if (state == 3) {LEFT();}
  if (state == 4) {UP();}
  
  if (command.length() == 0) return;
  
  // normal commands
  if (command == "0:0") {state = 0; return;}
  if (command == "1:0") {state = 1; return;}
  if (command == "-1:0") {state = 3; return;}
  if (command == "0:1") {state = 4; return;}
  if (command == "0:-1") {state = 2; return;}
  
  // ebanutie commands
  if (command == "1:10" || command == "up") {state = 4; return;}
  if (command == "1:11" || command == "down") {state = 2; return;}
  if (command == "1:12" || command == "right") {state = 1; return;}
  if (command == "1:13" || command == "left") {state = 3; return;}
  if (command == "1:14" || command == "stay" || command == "stop") {state = 0; return;}

  // control commands
  String delimiter = " ";
  String com = "";
  String arg = "";
  int del = command.indexOf(delimiter);
  
  if (del == -1) {
    com = command;
  } else {
    com = String(command);
    arg = String(command);
    com.remove(del, command.length() - del);
    arg.remove(0, del + 1);
  }

  if (com == "speed") {
    if (arg.length() != 0) {
      movement_speed = arg.toInt();
    } else {
      Serial.print(movement_speed);
    }
  }

  if (com == "test") {
    motorcheck();
  }

  if (com == "debug")
    if (arg.length() != 0) {
      debug = arg.toInt() > 0;
  } 

  if (com == "scenario")
    if (arg.length() != 0) {
      scenario_enable = arg.toInt() > 0;
  } 
}
