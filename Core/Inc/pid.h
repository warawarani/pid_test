/**
 * @file pid.h
 * @author warawarani (warawarani6201@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2024-04-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PID_H
#define PID_H
#define MAX_STEERING 1 /*ステアリングの数*/
/**
 * @brief pidを用いたステアリング出力計算用の構造体
 * 
 */
struct PID_CON_VAR{
  /*エラー値*/
  double error[2];
  double integral;
  //gain
  double KP,KI,KD;
  //interrupt period
  double DELTA_T;
  // max feedback val
  int MAXVAL;
  int powerPosition;
  //現在のエンコーダーの値
  float feedback_val;
  //目標値
  float target_val;
  //output
  double steeringPower;
  int tag;
};
//ステアリングの制御用構造体
struct PID_CON_VAR steerring[MAX_STEERING],*str[MAX_STEERING];

void InitPidParameter();
void ErrorValueCul(struct PID_CON_VAR *str);
float PidCul(struct PID_CON_VAR *str);
void SteeringPowerCul(struct PID_CON_VAR *str);

#endif