
#include <mbed.h>
#include "IMU.h"

namespace IMU{

    // anonymous namespace, acts as private data
    namespace {
        SPI* _spi;
        DigitalOut* _selMPU;
        DigitalOut* _selBMP;
        float Magnetometer_ASA[3];
        // uint8_t readAddr[1];
        uint8_t dataIn[22];

        volatile float beta = 0.6f;// sould be sqrt(3.0f / 4.0f) * 40.0f*deg2rad for 40°/s gyro error
        volatile float q0 = 0.9999f, q1 = 0.001f, q2 = 0.001f, q3 = 0.001f;	// quaternion of sensor frame relative to auxiliary frame
        volatile bool anglesComputed=false;
        
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;	// euler angles.
        uint32_t lastUpdate = 0; // used to calculate integration interval

        unsigned int ReadReg( uint8_t Addr, uint8_t Data=0x00 , bool toMPU=true)
        {
            unsigned int temp_val;
            if(toMPU)
                *_selMPU=0;
            else
                *_selBMP=0;

            wait_us(2);
            _spi->write(Addr);
            temp_val=_spi->write(Data);
            wait_us(2);

            if(toMPU)
                *_selMPU=1;
            else
                *_selBMP=1;

            return temp_val;
        }
        void  WriteReg( uint8_t Addr, uint8_t Data , bool toMPU)
        {
            ReadReg(Addr,Data, toMPU);
        }
        void ReadRegs( uint8_t Addr, uint8_t *ReadBuf, unsigned int Bytes , bool toMPU)
        {
            unsigned int  i = 0;

            if(toMPU)
                *_selMPU=0;
            else
                *_selBMP=0;
            

            wait_us(2);
            _spi->write(Addr | READ_FLAG);
            //_spi->write(NULL,0,ReadBuf,Bytes);
            for(i=0; i<Bytes; i++)
                ReadBuf[i] = _spi->write(0x00);
            wait_us(2);
            if(toMPU)
                *_selMPU=1;
            else
                *_selBMP=1;
        }

        void WriteRegs( uint8_t Addr, uint8_t *WriteBuf, unsigned int Bytes , bool toMPU)
        {
            unsigned int  i = 0;

            if(toMPU)
                *_selMPU=0;
            else
                *_selBMP=0;
            

            wait_us(2);
            _spi->write(Addr);
            for(i=0; i<Bytes; i++)
                _spi->write(WriteBuf[i]);
            wait_us(2);
            if(toMPU)
                *_selMPU=1;
            else
                *_selBMP=1;
        }

        // Fast inverse square-root
        // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

        float invSqrt(float x) {
            float halfx = 0.5f * x;
            float y = x;
            long i = *(long*)&y;
            i = 0x5f3759df - (i>>1);
            y = *(float*)&i;
            y = y * (1.5f - (halfx * y * y));
            //y = y * (1.5f - (halfx * y * y));//second NR step, more precise
            return y;
            // return 1.0/sqrt(x);
        }
    }

    void init(SPI* spi, DigitalOut* CSMPU, DigitalOut* CSBMP){
        _spi=spi;
        _selMPU=CSMPU;
        _selBMP=CSBMP;

        _spi->format(8,0);
        _spi->frequency(1000000);
        // readAddr[0] = 0x3B| READ_FLAG;
        // initial setup
        uint8_t MPU_Init_Data[16][2] = {
            {0x08, MPUREG_PWR_MGMT_1},     // Reset Device
            {0x30, MPUREG_USER_CTRL},       // I2C Master mode + I2C slave disable -> SPI (must do that quickly)
            {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
            //{0b00000000,MPUREG_SMPLRT_DIV}, // full sample rate
            {BITS_DLPF_CFG_98HZ, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 
            {0b00001000, MPUREG_GYRO_CONFIG},    // +-500dps
            {0b00001000, MPUREG_ACCEL_CONFIG},   // +-4G
            //{0b00000000, MPUREG_ACCEL_CONFIG_2}, // BW218 Hz

            {0x09, MPUREG_I2C_MST_CTRL},   // I2C Speed 500 kHz
            {0x81, MPUREG_I2C_MST_DELAY_CTRL}, //  slave 0 enable sample skip
            {0x09, MPUREG_I2C_SLV4_CTRL}, // in SLV4CONTROL is the general ext sample skip! 9 skip
            
            
            {0x0C, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
            //WRITE TO SLV0 REGISTER
            {0x0B, MPUREG_I2C_SLV0_REG}, //address : CNTL2
            {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
            {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

            {0x0A, MPUREG_I2C_SLV0_REG}, //address : CNTL1
            {0x12, MPUREG_I2C_SLV0_DO}, // measure at 8Hz in 16bit
            {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        };

        for(int i=0; i<16; i++) {
            WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0],true);
            wait(0.001);  //let time for CS to go up and down between writes
        }
        // read magnetometer calib
        WriteReg(MPUREG_I2C_SLV0_ADDR,0x0C|READ_FLAG,true); //Set the I2C slave addres of AK8963 and set for read.
        wait(0.001);  //let time for CS to go up and down between writes
        WriteReg(MPUREG_I2C_SLV0_REG, 0x10,true); //I2C slave 0 register address from where to begin data transfer
        wait(0.001);  //let time for CS to go up and down between writes
        WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83,true); //Read 3 bytes from the magnetometer
        wait(0.001); // wait for MPU to poll the slave
        uint8_t dataMagn[3];
        ReadRegs(MPUREG_EXT_SENS_DATA_00,dataMagn,3,true);
        // calibration factors
        for(int i = 0;i<3;i++){
            Magnetometer_ASA[i]=((float)dataMagn[i])*0.00390625+0.5;
        }
        
        // poll the magnetometer at the skipped FS rate, 
        wait(0.001);  //let time for CS to go up and down between writes
        WriteReg(MPUREG_I2C_SLV0_REG, 0x03,true); //I2C slave 0 register address from where to begin data transfer
        wait(0.001);  //let time for CS to go up and down between writes
        WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87,true); //Read 7 bytes from the magnetometer (last is latch)
    }

    void getBMPData(float* table){
        ReadRegs(0x3B| READ_FLAG,dataIn,20,true);

        // reformat data 
        
        //accel in G (we could actually not do the scaling, the result is normed after)
        table[0]= ((float)(__REVSH(*(int16_t*)(dataIn))))*1.220703125e-4f;
        table[1]= ((float)(__REVSH(*(int16_t*)(dataIn+2))))*1.220703125e-4f;
        table[2]= ((float)(__REVSH(*(int16_t*)(dataIn+4))))*1.220703125e-4f;

        //gyro in rad/s
        table[3]= ((float)(__REVSH(*(int16_t*)(dataIn+8))))*2.6646248122052432e-4f;
        table[4]= ((float)(__REVSH(*(int16_t*)(dataIn+10))))*2.6646248122052432e-4f;
        table[5]= ((float)(__REVSH(*(int16_t*)(dataIn+12))))*2.6646248122052432e-4f;

        //magn in µT (we could actually not do the scaling, the result is normed after)
        table[6]= (float)(*(int16_t*)(dataIn+14))*0.15f*Magnetometer_ASA[0];
        table[7]= (float)(*(int16_t*)(dataIn+16))*0.15f*Magnetometer_ASA[1];
        table[8]= (float)(*(int16_t*)(dataIn+18))*0.15f*Magnetometer_ASA[2];


    }

    void calibrateSensor(DigitalOut* led){
        *led=0;
        int nmes=0;
        // store gyro data
        int32_t tol = 50;
        int32_t data[3];
        int32_t sum[3] = {0,0,0};
        int32_t lastRead[3] = {0,0,0};
        //reset offset
        for(int i=0;i<6;i++){
            WriteReg(19+i, 0x00,true);
            wait(0.001);  //let time for CS to go up and down between writes
        }
        
        while (nmes<200){
            //read only gyro data
            ReadRegs(0x43| READ_FLAG,dataIn,6,true);

            data[0]= __REVSH(*(int16_t*)(dataIn));
            data[1]= __REVSH(*(int16_t*)(dataIn+2));
            data[2]= __REVSH(*(int16_t*)(dataIn+4));
            if((lastRead[0]-data[0])<tol && (lastRead[0]-data[0])>-tol &&(lastRead[1]-data[1])<tol &&(lastRead[1]-data[1])>-tol &&(lastRead[2]-data[2])<tol &&(lastRead[2]-data[2])>-tol ){
                sum[0] +=data[0];
                sum[1] +=data[1];
                sum[2] +=data[2];
                
                nmes++;
            }
            else{
                *led=1;
                sum[0] =0;
                sum[1] =0;
                sum[2] =0;

                nmes=0;
                wait_ms(200);
                *led=0;
            }
            lastRead[0]=data[0];
            lastRead[1]=data[1];
            lastRead[2]=data[2];
            wait_ms(10);
        }

        *led=1;
        for(int i =0;i<3;i++){
            sum[i]=-(sum[i]/400);
            WriteReg(19+i*2, (uint8_t)((sum[i]>>8)&0x00FF),true);
            wait(0.001);  //let time for CS to go up and down between writes
            WriteReg(20+i*2, (uint8_t)(sum[i]&0x00FF),true);
            wait(0.001);  //let time for CS to go up and down between writes
        }

        lastUpdate=us_ticker_read();// initialize time interval with a non 0 value

    }

    void printsyncData(float* data,Serial* s){
        s->printf("data from MPUsync : ");

        for(int i = 0; i<9;i++){
            s->printf("%f |",data[i]);
        }
        s->printf("\r\n");
    }

    //quaternion update based on sensor data
    void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, uint32_t now) {
        float recipNorm,deltat;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3,_4q0, _4q1, _4q2 ,_8q1, _8q2;
        bool magON=true;
        
        deltat = ((float)(now - lastUpdate))*1e-6f; // set integration time by time elapsed since last filter update
        lastUpdate = now;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        magON=!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f));        
        
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;   

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;


            if(magON){
                // Normalise magnetometer measurement
                recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;

                //mag specific variables
                _2q0mx = 2.0f * q0 * mx;
                _2q0my = 2.0f * q0 * my;
                _2q0mz = 2.0f * q0 * mz;
                _2q1mx = 2.0f * q1 * mx;
                _2q0q2 = 2.0f * q0 * q2;
                _2q2q3 = 2.0f * q2 * q3;
                q0q1 = q0 * q1;
                q0q2 = q0 * q2;
                q0q3 = q0 * q3;
                q1q2 = q1 * q2;
                q1q3 = q1 * q3;
                q2q3 = q2 * q3;


                // Reference direction of Earth's magnetic field
                hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
                hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
                _2bx = sqrt(hx * hx + hy * hy);
                _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
                _4bx = 2.0f * _2bx;
                _4bz = 2.0f * _2bz;

                // Gradient decent algorithm corrective step
                s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            }
            else{
                // IMU only specific variables
                _4q0 = 4.0f * q0;
                _4q1 = 4.0f * q1;
                _4q2 = 4.0f * q2;
                _8q1 = 8.0f * q1;
                _8q2 = 8.0f * q2;

                // Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
                s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            }

            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * deltat;
        q1 += qDot2 * deltat;
        q2 += qDot3 * deltat;
        q3 += qDot4 * deltat;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        anglesComputed = false;
    }

    // eular angles as 3 floats in degrees ; roll;pitch;yaw
    void getEulerAngles(float* angles){
        if(!anglesComputed){
            roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*RAD2DEG;
            pitch = asinf(-2.0f * (q1*q3 - q0*q2))*RAD2DEG;
            yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*RAD2DEG;
            anglesComputed = true;
        }
        angles[0]=roll;
        angles[1]=pitch;
        angles[2]=yaw;
    }
}



