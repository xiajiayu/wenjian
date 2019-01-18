#include "pid.h"
#include "stdlib.h"

using namespace std;

PID_Control::PID_Control()
{
    PID_Init();
}
void PID_Control::PID_X_Pos_Init(PID_Struct * P)
{
    P->P =1.0f;//略微大了点0.014f;//小了点0.013f;//0.015f;
    P->I =0.2f;
    P->D =0.0f;//2.80f;//2.40f;//2.10f;//0.85~0.95
    P->IMax = 50.0f;
    P->SetPoint = 0.0f;  //水平左右方向的偏移量
}

void PID_Control::PID_Y_Pos_Init(PID_Struct * P)
{
	P->P = 0.1f;//略微大了点0.014f;//小了点0.013f;//0.015f;
	P->I = 0.0f;
	P->D = 0.0f;//2.80f;//2.40f;//2.10f;//0.85~0.95
	P->IMax = 50.0f;
	P->SetPoint = 50.0f; //前后偏移量
}

void PID_Control::PID_X_CLear(PID_Struct * P)
{
    P->LastError=0;
    P->PreError=0;
    P->SumError=0;
    P->dError=0;
}
void PID_Control::PID_Y_CLear(PID_Struct * P)
{
	P->LastError = 0;
	P->PreError = 0;
	P->SumError = 0;
	P->dError = 0;
}

void PID_Control::PID_Init()
{
    //PID初始化
    PID_X_Pos_Init(&POS_x);
    PID_Y_Pos_Init(&POS_y);
}

float PID_Control::PID_Calc(PID_Struct *P, float ActualValue)
{

    P->PreError = P->SetPoint + ActualValue;
    P->dError = P->PreError - P->LastError;

    P->SumError += P->PreError;
    P->LastError = P->PreError;

    if(P->SumError >= P->IMax)
        P->SumError = P->IMax;
    else if(P->SumError <= -P->IMax)
        P->SumError = -P->IMax;

    P->POut = P->P * P->PreError;
    P->IOut = P->I * P->SumError;
    P->DOut = P->D * P->dError;
    return P->POut+P->IOut+P->DOut;
}

void PID_Control::shift_x(float &x_position, float &X_OUT)
{
    float x_offset=POS_x.SetPoint-x_position;

    cout<<"Begin adjust Position"<<endl;
//   usleep(50000);
//   if(abs(x_offset)<30 && abs(y_offset)<30) ThreShold = 56;
//   else if(abs(x_offset)>56 || abs(y_offset)>56) ThreShold=28;
//   全局变量
    X_OUT=abs(x_offset)<ThreShold?0:LIMIT_MIN_MAX(PID_Calc(&POS_x,x_position),-120,120);

    if(abs((char)X_OUT)<5){
        stable++;
    }else
        stable=0;
}




