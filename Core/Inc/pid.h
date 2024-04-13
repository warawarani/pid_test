#ifndef PID_H
#define PID_H
#define MAX_STEERING 1 //number of steering
/*variable of steering*/
struct PID_CON_VAR{
  double error[2];
  double integral;
  double KP,KI,KD;//gain
  double DELTA_T;//interrupt period
  int MAXVAL;//max feedback val
  int powerPosition;//using Locked Anti-Phase
  float feedback_val;
  float target_val;
  double steeringPower;//output
  int tag;
};
struct PID_CON_VAR steerring[MAX_STEERING],*str[MAX_STEERING];

void InitPidParameter();
float ErrorValueCul(struct PID_CON_VAR *str);//error cul
float PidCul(struct PID_CON_VAR *str);//pid cul
float SteeringPowerCul(struct PID_CON_VAR *str);//useing Locked Anti-Phase 

#endif