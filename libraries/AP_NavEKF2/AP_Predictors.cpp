
#include "AP_Predictors.h"
#include <AP_HAL.h>

#include <stdio.h>

//sean
#include <iostream>
using namespace std;
#include <stdlib.h>
//end sean

extern const AP_HAL::HAL& hal;

AP_Predictors::AP_Predictors()
{
    //ctor
}

//AP_Predictors::~AP_Predictors()
//{
//    //dtor
//}

void AP_Predictors::AttitudePredictor(Vector3f dAngIMU, Vector3f gyro_bias, AP_Int16 _msecPosDelay, Quaternion quat)
{
// retreive previous calculations
// D_T = D;

imuSampleTime_ms = hal.scheduler->millis();

D_q = D_q_k1;

tilde_q = dAngIMU - gyro_bias;

n_tilde_q = sqrt(tilde_q[0]*tilde_q[0]+tilde_q[1]*tilde_q[1]+tilde_q[2]*tilde_q[2]);
if (n_tilde_q < 1e-12f)
{
    delta_q[0] = 1;
    delta_q[1] = 0;
    delta_q[2] = 0;
    delta_q[3] = 0;
}
else
{
// check: delta_q=single([cos(0.5*n_tilde_q);(sin(0.5*n_tilde_q)/n_tilde_q).*tilde_q]);
    delta_q[0] = cosf(0.5f * n_tilde_q);
    rotScaler = (sinf(0.5f * n_tilde_q)) / n_tilde_q;
    delta_q[1] = tilde_q.x * rotScaler;
    delta_q[2] = tilde_q.y * rotScaler;
    delta_q[3] = tilde_q.z * rotScaler;
}

// update the quaternions by rotating from the previous attitude through
// the delta angle rotation quaternion
D_q_k1[0] = D_q[0]*delta_q[0] - D_q[1]*delta_q[1] - D_q[2]*delta_q[2] - D_q[3]*delta_q[3];
D_q_k1[1] = D_q[0]*delta_q[1] + D_q[1]*delta_q[0] + D_q[2]*delta_q[3] - D_q[3]*delta_q[2];
D_q_k1[2] = D_q[0]*delta_q[2] + D_q[2]*delta_q[0] + D_q[3]*delta_q[1] - D_q[1]*delta_q[3];
D_q_k1[3] = D_q[0]*delta_q[3] + D_q[3]*delta_q[0] + D_q[1]*delta_q[2] - D_q[2]*delta_q[1];

n_D_q_k1 = D_q_k1[0]*D_q_k1[0]+D_q_k1[1]*D_q_k1[1]+D_q_k1[2]*D_q_k1[2]+D_q_k1[3]*D_q_k1[3];
D_q_k1[0] = D_q_k1[0]/n_D_q_k1;
D_q_k1[1] = D_q_k1[1]/n_D_q_k1;
D_q_k1[2] = D_q_k1[2]/n_D_q_k1;
D_q_k1[3] = D_q_k1[3]/n_D_q_k1;

    if (imuSampleTime_ms - lastDStoreTime_ms >= 10) {
        lastDStoreTime_ms = imuSampleTime_ms;
        if (storeIndexD > (BUFFER_SIZE-1)) {
            storeIndexD = 0;
        }
        storedD_s[storeIndexD]=D_q_k1[0];
        D_q_tmp[0]=D_q_k1[1];
        D_q_tmp[1]=D_q_k1[2];
        D_q_tmp[2]=D_q_k1[3];
        storedD_v[storeIndexD] = D_q_tmp;
        DTimeStamp[storeIndexD] = lastDStoreTime_ms;
        storeIndexD = storeIndexD + 1;

//printf("%u , %u \n", storeIndexIMU, angRateTimeStamp[storeIndexIMU]);
//        cout << imuSampleTime_ms << "   " << storedAngRate[storeIndexIMU] << "\n";
    }

    int timeD;
    uint32_t bestTimeD = 200;
    uint16_t bestStoreIndex = 0;
    Quaternion D_Delay;
    bestStoreIndex = 0;

    for (uint16_t i=0; i<=(BUFFER_SIZE-1); i++)
    {
        timeD = abs( (int) imuSampleTime_ms- DTimeStamp[i] - constrain_int16(_msecPosDelay, 0, MAX_MSDELAY));
        //       printf("%u \n",_msecPosDelay);
	if (timeD < bestTimeD)
        {
            bestStoreIndex = i;
            bestTimeD = timeD;
        }
    }

D_Delay[0] = storedD_s[bestStoreIndex];
D_q_tmp=storedD_v[bestStoreIndex];
D_Delay[1] = D_q_tmp[0];
D_Delay[2] = D_q_tmp[1];
D_Delay[3] = D_q_tmp[2];

// D_q_delay^{-1}
q_tmp[0]= D_Delay[0];
q_tmp[1]= -D_Delay[1];
q_tmp[2]= -D_Delay[2];
q_tmp[3]= -D_Delay[3];

// D_q_delay^{-1} \times D_q

q_hat[0] = q_tmp[0]*D_q[0] - q_tmp[1]*D_q[1] - q_tmp[2]*D_q[2] - q_tmp[3]*D_q[3];
q_hat[1] = q_tmp[0]*D_q[1] + q_tmp[1]*D_q[0] + q_tmp[2]*D_q[3] - q_tmp[3]*D_q[2];
q_hat[2] = q_tmp[0]*D_q[2] + q_tmp[2]*D_q[0] + q_tmp[3]*D_q[1] - q_tmp[1]*D_q[3];
q_hat[3] = q_tmp[0]*D_q[3] + q_tmp[3]*D_q[0] + q_tmp[1]*D_q[2] - q_tmp[2]*D_q[1];

// q_hat_tau \times D_q_delay^{-1} \times D_q

q_tmp[0] = quat[0]*q_hat[0] - quat[1]*q_hat[1] - quat[2]*q_hat[2] - quat[3]*q_hat[3];
q_tmp[1] = quat[0]*q_hat[1] + quat[1]*q_hat[0] + quat[2]*q_hat[3] - quat[3]*q_hat[2];
q_tmp[2] = quat[0]*q_hat[2] + quat[2]*q_hat[0] + quat[3]*q_hat[1] - quat[1]*q_hat[3];
q_tmp[3] = quat[0]*q_hat[3] + quat[3]*q_hat[0] + quat[1]*q_hat[2] - quat[2]*q_hat[1];

q_tmp.normalize();
q_hat=q_tmp;

printf("test \n");

static FILE *mylog;
if (mylog==NULL){
	mylog = fopen("logtmp2.txt", "w");
}
if (mylog!=NULL){
// fputs ("Ali",  mylog);
fprintf (mylog, "{%u;%f;%f;%f;%f}\n", hal.scheduler->millis(), q_hat[0], q_hat[1],q_hat[2],q_hat[3]);
}
}
