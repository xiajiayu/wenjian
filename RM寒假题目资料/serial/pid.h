#ifndef PID_H
#define PID_H

#include "iostream"
#include <cmath>

#define LIMIT_MIN_MAX(x,min,max)   (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ConvertToAngle(x) ((x)*180.0/3.14159)

typedef struct PID{
        float SetPoint;

        float P;
        float I;
        float D;

        float LastError;
        float PreError;
        float SumError;
        float dError;

        float IMax;

        float POut;
        float IOut;
        float DOut;
}PID_Struct;

class PID_Control
{
public:
    PID_Control();
    void PID_Init();
    void PID_X_CLear(PID_Struct * P);
    void PID_Y_CLear(PID_Struct * P);

	float PID_Calc(PID_Struct *P, float ActualValue);
    void shift_x(float &x_position,float &X_OUT);

private:
    void PID_X_Pos_Init(PID_Struct * P);
    void PID_Y_Pos_Init(PID_Struct * P);

public:
    int stable;

//private:
    PID_Struct POS_x;
    PID_Struct POS_y;
    int ThreShold = 3;
};


#endif // ACTIONDEMO_H
