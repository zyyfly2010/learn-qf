#ifndef AP_PREDICTORS_H
#define AP_PREDICTORS_H

#include <AP_Math.h>
#include <AP_Param.h>
#include <vectorN.h>
//#include <matrix3.h>
//#include <quaternion.h>

#define BUFFER_SIZE  200   // sean buffer size for sensors
#define MAX_MSDELAY  2000   // maximum allowed delay

class AP_Predictors
{
    public:
        AP_Predictors();
        void AttitudePredictor(Vector3f dAngIMU, Vector3f gyro_bias, AP_Int16 _msecPosDelay, Quaternion quat);
    //    virtual ~AP_Predictors();
//        typedef float ftype;
//    #if MATH_CHECK_INDEXES
//        typedef VectorN<ftype,2> Vector2;
//        typedef VectorN<ftype,3> Vector3;
//        typedef VectorN<ftype,6> Vector6;
//        typedef VectorN<ftype,8> Vector8;
//        typedef VectorN<ftype,11> Vector11;
//        typedef VectorN<ftype,13> Vector13;
//        typedef VectorN<ftype,14> Vector14;
//        typedef VectorN<ftype,15> Vector15;
//        typedef VectorN<ftype,22> Vector22;
//        typedef VectorN<VectorN<ftype,3>,3> Matrix3;
//        typedef VectorN<VectorN<ftype,22>,22> Matrix22;
//        typedef VectorN<VectorN<ftype,50>,22> Matrix22_50;
//    #else
//        typedef ftype Vector2[2];
//        typedef ftype Vector3[3];
//        typedef ftype Vector6[6];
//        typedef ftype Vector8[8];
//        typedef ftype Vector11[11];
//        typedef ftype Vector13[13];
//        typedef ftype Vector14[14];
//        typedef ftype Vector15[15];
//        typedef ftype Vector22[22];
//        typedef ftype Vector31[31];
//        typedef ftype Matrix3[3][3];
//        typedef ftype Matrix22[22][22];
//        typedef ftype Matrix31_50[31][50];
//    #endif

    private:
        uint32_t imuSampleTime_ms;
        Matrix3f D;
        Matrix3f D_T;
        Quaternion D_q;
        Quaternion D_q_k1;
        float n_D_q_k1;
        Quaternion q_hat;   //sean prediction of the current quaternion
        Quaternion q_hat_T_k1;
        Vector3f tilde_q;
        float n_tilde_q;
        Quaternion delta_q;
        Matrix3f R_hat_T;
        Matrix3f R_hat;
        Vector3f d_v;
        Vector3f v_hat; // prediction of current velocity
        Vector3f d_p;
        Vector3f p_hat; // prediction of current position
        Vector3f v_hat_m; // prediction of current velocity mixed-invariant
        Vector3f d_p_m;
        Vector3f p_hat_m; // prediction of current position mixed-invariant
        Vector3f d_v_m;  // mixed-invariant

        uint16_t storeIndexIMU;						// arash
        uint32_t lastAngRateStoreTime_ms;		 // sean
        VectorN<Vector3f,BUFFER_SIZE> storedAngRate;       //  sean
        VectorN<Vector3f,BUFFER_SIZE> storeddVelIMU1; //sean
        VectorN<Vector3f,BUFFER_SIZE>  storeddVelIMU2; //sean
        uint32_t angRateTimeStamp[BUFFER_SIZE];    		  // sean

        uint16_t storeIndexMag;
        uint32_t lastMagStoreTime_ms;
        VectorN<Vector3f,BUFFER_SIZE> storedMag;       //  sean
        uint32_t MagTimeStamp[BUFFER_SIZE];    		  // sean


        uint16_t storeIndexTas;              // sean
        uint32_t lastTasStoreTime_ms;           // sean
        float storedTas[BUFFER_SIZE];                 // sean
        uint32_t TasTimeStamp[BUFFER_SIZE];    		  // sean

        uint16_t storeIndexHgt;              // sean
        uint32_t lastHgtStoreTime_ms;           // sean
        float storedHgt[BUFFER_SIZE];                 // sean
        uint32_t HgtTimeStamp[BUFFER_SIZE];    		  // sean
        uint32_t lastHealthyHgtTime_ms; // Sean time the barometer was last declared healthy

        uint16_t storeIndexD;						// sean
        uint32_t lastDStoreTime_ms;		 // sean
        VectorN<Vector3f,BUFFER_SIZE> storedD_v;       //  sean vector part of quaternion Delta
        float storedD_s[BUFFER_SIZE];                 // sean  scalar part of quaternion Delta
        uint32_t DTimeStamp[BUFFER_SIZE];    		  // sean
        VectorN<Vector3f,BUFFER_SIZE> storedd_v;       //  sean buffer for delta corrsponding to velocity prediction
        VectorN<Vector3f,BUFFER_SIZE> storedd_p;       //  sean buffer for delta corrsponding to position prediction
        VectorN<Vector3f,BUFFER_SIZE> storedd_p_m;       //  sean buffer for delta corrsponding to position prediction mixed-invariant
        VectorN<Vector3f,BUFFER_SIZE> storedd_v_m;       //  sean buffer for delta corrsponding to velocity prediction mixed-invariant
        uint32_t ctr_rst;  // reset predictor cntr

        float init_reset;   //sean reset initial quaternions

        float rotScaler;
        Vector3f D_q_tmp;
        Quaternion q_tmp;
};

#endif // AP_PREDICTORS_H
