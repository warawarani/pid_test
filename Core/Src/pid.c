#include "pid.h"

/*setting pid parameter*/
void InitPidParameter(){
  int count;
  for(count=0;count<MAX_STEERING;count++){
    str[count] = &steerring[count];
    str[count] -> KP =1;//P gain
    str[count] -> KI =1;//I gain
    str[count] -> KD =1;//D gain
    str[count] -> DELTA_T =0.0001;//interrupt period
    str[count] -> MAXVAL =2000;//max feedback val
    str[count] -> powerPosition = str[count]->MAXVAL/2;
    str[count] -> tag = count;
  }
}

/*culculating error value*/
float ErrorValueCul(struct PID_CON_VAR *str)
{
  int halfPoint;
  float errorVal;
  if(str->target_val<=str->MAXVAL/2){
    halfPoint=str->target_val + str->MAXVAL/2;
    if(str->feedback_val<=halfPoint){
      errorVal=str->feedback_val - str->target_val;
    }
    else if(str->feedback_val>halfPoint){
      errorVal=-str->target_val - str->MAXVAL + str->feedback_val;
    }
  }
  else if(str->target_val>str->MAXVAL/2){
    halfPoint=str->target_val - str->MAXVAL /2;
    if(str->feedback_val>=halfPoint){
      errorVal=str->feedback_val - str->target_val;
    }
    else if(str->feedback_val<halfPoint){
      errorVal= -str->target_val + str->MAXVAL +str->feedback_val;
    }
  }
  str->error[1]=errorVal;
}

/*calculating pid value*/
float PidCul(struct PID_CON_VAR *str)
{

float p, i, d;

str->error[0] = str->error[1];
str->integral += (str->error[1] + str->error[0]) / 2.0 * str->DELTA_T;

p = str->KP * str->error[1];
i = str->KI * str->integral;
d = str->KD * (str->error[1] - str->error[0])/str->DELTA_T;

return (p + i + d);

}

/*culculating output power*/
float SteeringPowerCul(struct PID_CON_VAR *str)
{
  ErrorValueCul(&str[str->tag]);
  str->steeringPower= str->powerPosition + PidCul(&str[str->tag]);
}