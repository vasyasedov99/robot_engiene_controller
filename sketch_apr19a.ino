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

int gain_fl = 100;
int gain_fr = 100;
int gain_bl = 100;
int gain_br = 100;

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

class speedKeeper {
public:
  encCounter *ec;
  GMotor *motor;

  // motor static parameters
  int friction = 20;
  float power_scale = 1;
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
  int target_speed = 0;
  long target_pos = 0;

  // self-handling variables
  bool controlled = false;
  int mtimer = 100; // timer between engiene updates
  long ms = 0;      // time in the last update
  
  speedKeeper(encCounter *ec, GMotor *motor) {
    this->ec = ec;
    this->motor = motor;
    ms = millis();
  }

  // interface method
  void keep(int target_spd) {
    target_speed = target_spd;
    long cticks = ec->update(1);
    if (tick_previous != 0 && cticks > tick_previous) {
      ticks += cticks - tick_previous;
    }
    
    tick_previous = cticks;
    
    if (!controlled)
    if (millis() - ms > mtimer) {
      makestep(millis() - ms);
      ms = millis();
    }
  }

  // tick calculating
  void tick(int mtimer) {
      speed = ticks * 1000 / mtimer;
      pos += ticks;

      speed_register(speed);
      ticks = 0;
  }

  //calculate and send motor value
  void motor_handle() {
    int speed_difference = speed - speed_history[0];
    gain = max((target_pos - pos) * 0.7 + (target_speed - speed) * 0.3, 0) * 5;
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

void UP(){
  motorFL.setMode(FORWARD);
  motorFL.setSpeed(gain_fl);
  motorFR.setMode(FORWARD);
  motorFR.setSpeed(gain_fr);
  motorBL.setMode(FORWARD);
  motorBL.setSpeed(gain_bl);
  motorBR.setMode(FORWARD);
  motorBR.setSpeed(gain_br);
  }
  
  void DOWN(){
  motorFL.setMode(BACKWARD);
  motorFL.setSpeed(gain_fl);
  motorFR.setMode(BACKWARD);
  motorFR.setSpeed(gain_fr);
  motorBL.setMode(BACKWARD);
  motorBL.setSpeed(gain_bl);
  motorBR.setMode(BACKWARD);
  motorBR.setSpeed(gain_br);
  }
  void RIHT(){
  motorFL.setMode(FORWARD);
  motorFL.setSpeed(gain_fl);
  motorFR.setMode(BACKWARD);
  motorFR.setSpeed(gain_fr);
  motorBL.setMode(BACKWARD);
  motorBL.setSpeed(gain_bl);
  motorBR.setMode(FORWARD);
  motorBR.setSpeed(gain_br);
    
  }
  void LEFT(){       
  motorFL.setMode(BACKWARD);
  motorFL.setSpeed(gain_fl);
  motorFR.setMode(FORWARD);
  motorFR.setSpeed(gain_fr);
  motorBL.setMode(FORWARD);
  motorBL.setSpeed(gain_bl);
  motorBR.setMode(BACKWARD);
  motorBR.setSpeed(gain_br);
  }

  void STAY(){       
  motorFL.setMode(STOP);
  motorFR.setMode(STOP);
  motorBL.setMode(STOP);
  motorBR.setMode(STOP);
  }
  

void setup() {
  Serial.begin(9600);
  Serial.println("front left");
  motorFL.setMode(AUTO);
  motorFL.setSpeed(100);
  //delay(1000);
  Serial.println("front right");
  motorFR.setMode(AUTO);
  motorFR.setSpeed(65);
  //delay(1000);
  Serial.println("back left");
  motorBL.setMode(AUTO);
  motorBL.setSpeed(65);
  //delay(1000);
  Serial.println("back right");
  motorBR.setMode(AUTO);
  motorBR.setSpeed(65);
    delay(1000);
   motorFL.setMode(STOP);
  motorFR.setMode(STOP);
  motorBL.setMode(STOP);
  motorBR.setMode(STOP);
    
}

void loop() {
  //Serial.println(encBL.update(1));
  String command = "0:1";//Serial.readStringUntil('\n');
  if (command.length() == 0) return;
  
  // normal commands
  if (command == "0:0") {STAY(); return;}
  if (command == "1:0") {RIHT(); return;}
  if (command == "-1:0") {LEFT(); return;}
  if (command == "0:1") {
    motorFL.setMode(BACKWARD);
    speedFL.keep(25);
    //for(int i = 0; i < speedFL.cspeed; i++) Serial.print("#");
    //Serial.println();
    motorFR.setMode(FORWARD);
    speedFR.keep(25);
    motorBR.setMode(BACKWARD);
    speedBR.keep(25);
    motorBL.setMode(FORWARD);
    speedBL.keep(25);
    //UP(); 
    return;
    }
  if (command == "0:-1") {DOWN(); return;}
  
  // ebanutie commands
  if (command == "1:10") {UP(); return;}
  if (command == "1:11") {DOWN(); return;}
  if (command == "1:12") {RIHT(); return;}
  if (command == "1:13") {LEFT(); return;}
  if (command == "1:14") {STAY(); return;}

  // control commands
  String delimiter = " ";
  String com = "";
  String arg = "";
  int del = command.indexOf(delimiter);
  Serial.print(command);
  Serial.print(":");
  if (del == -1) {
    com = command;
  } else {
    com = String(command);
    arg = String(command);
    com.remove(del, command.length() - del);
    arg.remove(0, del + 1);
  }
  Serial.print(com);
  Serial.print("-");
  Serial.print(arg);
  Serial.print("; ");

  if (com == "fl") {
    int gain = arg.toInt();
    Serial.print(gain);
    gain_fl = gain;
  }
  if (com == "fr") {
    int gain = arg.toInt();
    Serial.print(gain);
    gain_fr = gain;
  }
  if (com == "bl") {
    int gain = arg.toInt();
    Serial.print(gain);
    gain_bl = gain;
  }
  if (com == "br") {
    int gain = arg.toInt();
    Serial.print(gain);
    gain_br = gain;
  }
  if (com == "gain") {
    Serial.print(gain_fl);
    Serial.print("-");
    Serial.print(gain_fr);
    Serial.print("-");
    Serial.print(gain_bl);
    Serial.print("-");
    Serial.print(gain_br);
  }
}
