/****************************************************************************
* �� �� ��: main.c
* ��    ��: Amo [ www.amoMcu.com ��Ī��Ƭ��]
* ��    ��: 2014-04-08
* ��    ��: 1.0
* ��    ��: mpu6050����������
****************************************************************************/

#include <ioCC2530.h>
#include <math.h>
#include "IIC.h"
#include "MPU6050.h"
#include "uart/hal_uart.h"

#define I2C_PORT_NUM        1
#define BUFSIZE             14  //ȷ�ϴ�С������



volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];
volatile uint32_t I2CReadLength[I2C_PORT_NUM];
volatile uint32_t I2CWriteLength[I2C_PORT_NUM];

int16_t mpu6050_gyro_x=0,mpu6050_gyro_y=0,mpu6050_gyro_z=0;
int16_t mpu6050_acc_x=0,mpu6050_acc_y=0,mpu6050_acc_z=0;
_Matrix Mat = {0};
_Attitude att = {0};
imu660_offset set = {0};
float Atmpe_Y,Atmpe_X;
imu660_data imu = {0} ;
#define kp      20.0f                        //proportional gain governs rate of convergence to accelerometer/magnetometer
#define ki      0.01f                     //integral gain governs rate of convergenceof gyroscope biases
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error
int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;                           // �����������˲�����      gyro (������)
int16_t acc_x = 0, acc_y = 0, acc_z = 0;                              // ������ٶȼ��˲�����     acc (accelerometer ���ٶȼ�)

char strTemp[64];

void mpu6050_get_gyro(void)
{
  MPU6050_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14);
  /*mpu6050_acc_x = (((int16_t)I2CSlaveBuffer[PORT_USED][0]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][1]); 
  mpu6050_acc_y = (((int16_t)I2CSlaveBuffer[PORT_USED][2]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][3]); 
  mpu6050_acc_z = (((int16_t)I2CSlaveBuffer[PORT_USED][4]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][5]); */
  
   mpu6050_gyro_x = (((int16_t)I2CSlaveBuffer[PORT_USED][8]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][9]); 
   mpu6050_gyro_y= (((int16_t)I2CSlaveBuffer[PORT_USED][10]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][11]); 
   mpu6050_gyro_z= (((int16_t)I2CSlaveBuffer[PORT_USED][12]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][13]); 
   //printf("%d,%d,%d\r",mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z);
   //sprintf(strTemp, "[%7d %7d %7d] : [%7d %7d %7d]\n\r", \
                    mpu6050_acc_x, mpu6050_acc_y, mpu6050_acc_z, mpu6050_gyro_x, mpu6050_gyro_y, mpu6050_gyro_z);

                

                //���ڴ�ӡ������
   //Uart0SendString(strTemp, strlen(strTemp)); 
  
}

void mpu6050_get_accdata(void)
{

    MPU6050_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14);

     mpu6050_acc_x = (((int16_t)I2CSlaveBuffer[PORT_USED][0]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][1]); 
  mpu6050_acc_y = (((int16_t)I2CSlaveBuffer[PORT_USED][2]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][3]); 
  mpu6050_acc_z = (((int16_t)I2CSlaveBuffer[PORT_USED][4]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][5]); 
    //printf("%d,%d,%d\n",mpu6050_acc_x,mpu6050_acc_y,mpu6050_acc_z);
}
/*
 * ��������get_iir_factor
 * ����  ����ȡIIR�˲������˲�����
 * ����  ��out_factor�˲������׵�ַ��Time����ִ�����ڣ�Cut_Off�˲���ֹƵ��
 * ����  ��
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
    *out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}
/* ��ȡIIR��ͨ�˲� */
void IIR_imu(void)
{
	unsigned char i;
        //printf("%d,%d,%d\r",set.gyro.x,set.gyro.y,set.gyro.z);
    for(i=0;i<=100;i++)
    {
        mpu6050_get_gyro();
        set.gyro.x+=mpu6050_gyro_x;
        set.gyro.y+=mpu6050_gyro_y;
        set.gyro.z+=mpu6050_gyro_z;
        //printf("%f,%f,%f\r",set.gyro.x,set.gyro.y,set.gyro.z);
    }
    set.gyro.x/= 100;
    set.gyro.y/= 100;
    set.gyro.z/= 100;
    set.offset_flag = 1;
    //printf("%f,%f,%f\r",set.gyro.x,set.gyro.y,set.gyro.z);
    //printf("%d",set.offset_flag);
    //send_a_byte(set.offset_flag);
    get_iir_factor(&imu.att_acc_factor,imu_Read_Time,15);
    get_iir_factor(&imu.att_gyro_factor,imu_Read_Time,10);
}
/**
  * @brief   IIR��ͨ�˲���
  * @param   *acc_in ������������ָ�����
  * @param   *acc_out �����������ָ�����
  * @param   lpf_factor �˲�����
  * @retval  x
  */
float iir_lpf(float in,float out,float lpf_factor)
{
    out = out + lpf_factor * (in - out);
    return out;
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*
 * ��������mahony_update
 * ����  ����̬����
 * ����  ���������������ݣ���λ������/�룩�����ٶ��������ݣ���λ��g��
 * ����  ��
 */
//Gyroscope units are radians/second, accelerometer  units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    if(ax*ay*az==0)
        return;
    gx=gx*(PI / 180.0f);
    gy=gy*(PI / 180.0f);
    gz=gz*(PI / 180.0f);
    //[ax,ay,az]�ǻ�������ϵ�¼��ٶȼƲ�õ���������(��ֱ����)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //VectorA = MatrixC * VectorB
    //VectorA ���ο���������ת���ڻ����µ�ֵ
    //MatrixC ����������ϵת��������ϵ����ת����
    //VectorB ���ο�����������0,0,1��
    //[vx,vy,vz]�ǵ�������ϵ����������[0,0,1]����DCM��ת����(C(n->b))����õ��Ļ�������ϵ�е���������(��ֱ����)

    vx = Mat.DCM_T[0][2];
    vy = Mat.DCM_T[1][2];
    vz = Mat.DCM_T[2][2];

    //��������ϵ��������˵õ�������������e���ǲ����õ���v����Ԥ��õ��� v^֮��������ת�������v������[ax,ay,az]��,v^����[vx,vy,vz]��
    //����������������DCM�������Ҿ���(����DCM�����е���Ԫ��)�������������þ��ǽ�bϵ��n��ȷ��ת��ֱ���غϡ�
    //ʵ����������������ֻ��bϵ��nϵ��XOYƽ���غ�����������z����ת��ƫ�������ٶȼ��޿��κΣ�
    //���ǣ����ڼ��ٶȼ��޷���֪z���ϵ���ת�˶������Ի���Ҫ�õشż�����һ��������
    //���������Ĳ���õ��Ľ��������������ģ������֮��н����ҵĳ˻�a��v=|a||v|sin��,
    //���ٶȼƲ����õ�������������Ԥ��õ��Ļ������������Ѿ�������λ����������ǵ�ģ��1��
    //Ҳ����˵���������Ĳ���������sin���йأ����ǶȺ�Сʱ�����������Խ����ڽǶȳ����ȡ�

    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    //������������л���
    exInt = exInt + ex*ki;
    eyInt = eyInt + ey*ki;
    ezInt = ezInt + ez*ki;

    //��̬���������ٶ��ϣ��������ٶȻ���Ư�ƣ�ͨ������Kp��Ki�������������Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
    gx = gx + kp*ex + exInt;
    gy = gy + kp*ey + eyInt;
    gz = gz + kp*ez + ezInt;

    //һ�����������������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)* MahonyPERIOD * 0.0005f;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)* MahonyPERIOD * 0.0005f;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)* MahonyPERIOD * 0.0005f;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)* MahonyPERIOD * 0.0005f;

    //��������������Ԫ�����й�һ�������õ������徭����ת����µ���Ԫ����
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    //printf("%f,%f,%f,%f\n",q0,q1,q2,q3);

    static float pitch_old=0, roll_old=0,yaw_old=0;
    float temp = 0;
    //��Ԫ��תŷ����
    imu.pitch =   atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * (180.0f / PI);
    imu.roll  =   -asin(2.0f*(q0*q2 - q1*q3)) * (180.0f / PI);
    //z����ٶȻ��ֵ�ƫ����
    imu.yaw += imu.deg_s.z  * MahonyPERIOD * 0.001f;//���ɣ�Ҳ������imu_Read_Time�������Լ����ſ���Ч��

    temp = imu.pitch-pitch_old;
    if(temp<2 && temp > -2)
    {
        imu.pitch = pitch_old;
    }
    else
    {
        pitch_old = imu.pitch;
    }
    //temp = imu.roll-roll_old;
    /*if(temp<2 && temp > -2)
    {
        imu.roll = roll_old;
    }
    else
    {
        roll_old = imu.roll;
    }*/
    temp = imu.yaw-yaw_old;
    if(temp<0.1f && temp > -0.1f)
    {
        imu.yaw = yaw_old;
    }
    else
    {
        yaw_old = imu.yaw;
    }
//    if(imu.yaw>180) imu.yaw-=360;
//    if(imu.yaw<-180) imu.yaw+=360;
}
/*
 * ��������rotation_matrix
 * ����  ����ת���󣺻�������ϵ -> ��������ϵ
 * ����  ��
 * ����  ��
 */
void rotation_matrix(void)
{
    Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

    Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);

    Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
    Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
    Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * ��������rotation_matrix_T
 * ����  ����ת�����ת�þ��󣺵�������ϵ -> ��������ϵ
 * ����  ��
 * ����  ��
 */
void rotation_matrix_T(void)
{
    Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2);

    Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);

    Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
    Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
    Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * ��������Matrix_ready
 * ����  ���������׼����Ϊ��̬����ʹ��
 * ����  ��
 * ����  ��
 */
void Matrix_ready(void)
{
    rotation_matrix();                      //��ת�������
    rotation_matrix_T();                    //��ת�������������
}
/***********************************************************
�������ƣ�void IMU(void)
�������ܣ������̬������ֵ
��ڲ�������
���ڲ�������
�� ע��ֱ�Ӷ�ȡimu.pitch  imu.roll  imu.yaw
***********************************************************/
void IMU(void)
{
    //printf("%d\r",set.offset_flag);
    if(set.offset_flag)
    {
        /*��ȡX��Y�Ľ��ٶȺͼ��ٶ�*/
        /*�˲��㷨*/
        mpu6050_get_gyro();
        mpu6050_get_accdata();
        //printf("%d,%d,%d\r",mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z);
        mpu6050_gyro_x-=set.gyro.x;
        mpu6050_gyro_y-=set.gyro.y;
        mpu6050_gyro_z-=set.gyro.z;
        
        

        acc_x = iir_lpf(mpu6050_acc_x,acc_x,imu.att_acc_factor);
        acc_y = iir_lpf(mpu6050_acc_y,acc_y,imu.att_acc_factor);
        acc_z = iir_lpf(mpu6050_acc_z,acc_z,imu.att_acc_factor);
        //printf("%d,%d,%d\r",acc_x,acc_y,acc_z);
        gyro_x =iir_lpf(mpu6050_gyro_x,gyro_x,imu.att_gyro_factor);
        gyro_y =iir_lpf(mpu6050_gyro_y,gyro_y,imu.att_gyro_factor);
        gyro_z =iir_lpf(mpu6050_gyro_z,gyro_z,imu.att_gyro_factor);
        //printf("%d,%d,%d\r",gyro_x,gyro_y,gyro_z);
        


        //=================����������
        /*acc_x = (float)mpu6050_acc_x * Acc_Gain * G;
        acc_y = (float)mpu6050_acc_y * Acc_Gain * G;
        acc_z = (float)mpu6050_acc_z * Acc_Gain * G;
        gyro_x = (float)mpu6050_gyro_x * Gyro_Gain;
        gyro_y = (float)mpu6050_gyro_y * Gyro_Gain;
        gyro_z = (float)mpu6050_gyro_z * Gyro_Gain;
        //-----------------IIR�˲�
        acc_x = acc_x*Kp_New + acc_x_old *Kp_Old;
        acc_y = acc_y*Kp_New + acc_y_old *Kp_Old;
        acc_z = acc_z*Kp_New + acc_z_old *Kp_Old;
        acc_x_old = acc_x;
        acc_y_old = acc_y;
        acc_z_old = acc_z;
        IMUupdate(acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,&imu);*/
        //===============================

        /*���ݴ洢*/
        imu.acc_g.x = (float)acc_x/4096; //���ٶȼ�����Ϊ:��8g/4096, ��16g/2048, ��4g/8192, ��2g/16384 
        imu.acc_g.y = (float)acc_y/4096;
        imu.acc_g.z = (float)acc_z/4096;
        imu.deg_s.x = (float)mpu6050_gyro_x/16.4f;//����������Ϊ:��2000dps/16.4, ��1000dps/32.8, ��500 dps /65.6
        imu.deg_s.y = (float)mpu6050_gyro_y/16.4f;//��250 dps/131.2, ��125 dps/262.4
        imu.deg_s.z = (float)mpu6050_gyro_z/16.4f;

        //��������̬����
        //imu.roll = -Kalman_Filter_x(imu.acc_g.x,imu.deg_s.x);
        //imu.pitch = -Kalman_Filter_y(imu.acc_g.y,imu.deg_s.y);
        //imu.yaw = -Kalman_Filter_x(imu.acc_g.z,imu.deg_s.z);

        /*��̬����*/
        mahony_update(imu.deg_s.x,imu.deg_s.y,imu.deg_s.z,imu.acc_g.x,imu.acc_g.y,imu.acc_g.z);
        Matrix_ready();
    }
}

void mpu6050_init(void)
{
    mpu6050_gyro_x=0;
    mpu6050_gyro_y=0;
    mpu6050_gyro_z=0;
    //�����˲�����
    IIR_imu();
}

void DelayI2C(uint32_t m)
{
  uint32_t i;
  
  for(; m != 0; m--)    
       for (i=0; i<5; i++);
}

void MPU6050_Read(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char readNum)
{
    unsigned char i; 
    for(i = 0; i<readNum; i++)
    {
        //��ĳ��IIC����������
        I2CSlaveBuffer[PORT_USED][i] = Read_Add(REG_Address + i, SlaveAddress);
    }
}

unsigned char MPU6050_Read_1BYTE(unsigned char SlaveAddress,unsigned char REG_Address)
{
    return Read_Add(REG_Address, SlaveAddress);
}


void MPU6050_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data) 
{
    //��ĳ��IIC����дָ���ĳ����ַ���������
    Write_Add(REG_Address, REG_data, SlaveAddress);
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0];  
//    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;   // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data;    // combine data with existing byte
    MPU6050_Write(slaveAddr,regAddr,tmp);   
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0];  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_Write(slaveAddr,regAddr,tmp); 
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted

#if 1 //  �����ȡ��ȥ�����ε�λ�������ƶ�
    uint8_t tmp;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0]; 
//    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
#else
    uint8_t tmp;
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0]; 
    *data = tmp;
#endif
}

void MPU6050_Initialize(void)
{
    //printf("%s",MPU6050_DEFAULT_ADDRESS);
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 
        MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,
        MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_2000);
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 
        MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_8);
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);//(DISABLE); 
}

uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;

    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, \
        MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    
    return tmp<<(MPU6050_WHO_AM_I_BIT-MPU6050_WHO_AM_I_LENGTH+1);  // ע���������λ  amomcu
}

//���ٶȲ���, �¶Ȳ����� �����ǲ���
void MPU6050_GetRawAccelGyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* temperature, int16_t* gx, int16_t* gy, int16_t* gz) 
{
//    u8 tmpBuffer[14]; 
    //int i;
    MPU6050_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14); 

    /* Get acceleration */
 //   for(i=0; i<3; i++) 
//      AccelGyro[i]=((int16_t)((uint16_t)I2CSlaveBuffer[PORT_USED][2*i] << 8) + I2CSlaveBuffer[PORT_USED][2*i+1]);
   /* Get Angular rate */
//    for(i=4; i<7; i++)
//      AccelGyro[i-1]=((int16_t)((uint16_t)I2CSlaveBuffer[PORT_USED][2*i] << 8) + I2CSlaveBuffer[PORT_USED][2*i+1]); 
    *ax = (((int16_t)I2CSlaveBuffer[PORT_USED][0]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][1]); 
    *ay = (((int16_t)I2CSlaveBuffer[PORT_USED][2]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][3]); 
    *az = (((int16_t)I2CSlaveBuffer[PORT_USED][4]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][5]); 
    
    /*mpu6050_acc_x = *ax;
    mpu6050_acc_y = *ay;
    mpu6050_acc_z = *az;*/


    // �¶�
    *temperature = (((int16_t)I2CSlaveBuffer[PORT_USED][6]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][7]); 
    
    *gx = (((int16_t)I2CSlaveBuffer[PORT_USED][8]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][9]); 
    *gy = (((int16_t)I2CSlaveBuffer[PORT_USED][10]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][11]); 
    *gz = (((int16_t)I2CSlaveBuffer[PORT_USED][12]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][13]); 
    
    /*mpu6050_gyro_x = *gx;
    mpu6050_gyro_y = *gy;
    mpu6050_gyro_z = *gz;*/
}
#define T 0.02//�������ڣ�0.02s
//����У׼���������
float ax_cali=0,ay_cali=0,az_cali=0;
short gx_cali=0,gy_cali=0,gz_cali=0;
/***********************
//������ٶȼ���������ã�50��ȡƽ����У׼�������뱣��ˮƽ
//����ֵ��0:У׼δ��ɣ�1��У׼���
//�Դ�������У������
**********************/
u8 Acc_Get_ErrData(short ax,short ay,short az,u8 enable)
{
	static u8 i=0;
	float norm;
	static s32 axcount=0;
	static s32 aycount=0;
	static s32 azcount=0;
	float array_x,array_y,array_z;
	if(enable!=1)
	{
		i=0;
		axcount=0;aycount=0;azcount=0;
		return 0;
	}
	else if(i<50)
	{
		axcount+=ax;aycount+=ay;azcount+=az;
		i++;
		return 0;
	}
	else
	{
		array_x=axcount/50.0;array_y=aycount/50.0;array_z=azcount/50.0;//ʵ������
		norm=Q_rsqrt(array_x*array_x+array_y*array_y+array_z*array_z);//����������һ��
		array_x*=norm;array_y*=norm;array_z*=norm;//ʵ�ʵ�λ����
		ax_cali-=array_x;//����Ϊ���У����������ʵ�ʵ�λ�����͵�λ��������֮���������
		ay_cali-=array_y;
		az_cali+=1-array_z;
		i=0;
		axcount=0;aycount=0;azcount=0;
		return 1;
	}
}
/***********************
//�����������������ã�50��ȡƽ����У׼�������뱣�־�ֹ
//����ֵ 0:У׼δ��ɣ�1��У׼���
**********************/
u8 Gyro_Get_ErrData(short gx,short gy,short gz,u8 enable)
{
	static u8 i=0;
	static s32 gxcount=0;
	static s32 gycount=0;
	static s32 gzcount=0;
	if(enable!=1)
	{
		i=0;
		gxcount=0;gycount=0;gzcount=0;
		return 0;
	}
	else if(i<50)
	{
		gxcount+=gx;gycount+=gy;gzcount+=gz;
		i++;
		return 0;
	}
	else
	{
		gxcount/=50;gycount/=50;gzcount/=50;
		gx_cali+=gxcount;
		gy_cali+=gycount;
		gz_cali+=gzcount;
		i=0;
		gxcount=0;gycount=0;gzcount=0;
		return 1;
	}
}
/***********************
//�ñ����������У�����ٶȼ�ԭʼ����
**********************/
void Acc_Calibrate(short *ax,short *ay,short *az)
{
	short accx=*ax;
	short accy=*ay;
	short accz=*az;
	float norm;
	//��У׼����У�����ٶȼ�ԭʼ����
	norm=my_sqrt(accx*accx+accy*accy+accz*accz);
	accx+=ax_cali*norm;
	accy+=ay_cali*norm;
	accz+=az_cali*norm;
	*ax = accx;
	*ay = accy;
	*az = accz;
}
/***********************
//����Ϊ�������˲�
**********************/
float sigma_a=0.01,sigma_g=0.01;//������������������ͼ��ٶȼƹ۲���������
//����������
float K_roll[2];
float K_pitch[2];
//��С����������,M[n|n]��M[n-1|n-1]
float mmse_roll[2][2] = { { 1, 0 },{ 0, 1 } };
float mmse_pitch[2][2] = { { 1, 0 },{ 0, 1 } };
//��СԤ�����������,M[n|n-1]
float mPmse_roll[2][2];
float mPmse_pitch[2][2];
//�����ںϿ������˲��㷨
void IMUupdate(short *gx,short *gy,short *gz,short ax,short ay,short az,float *roll,float *pitch,float *yaw)
{
	float temp;//Ϊ���ټ���������ʱ�������ݣ���ʵ������
	float roll_temp,pitch_temp;//״̬����Ԥ��ֵ,s[n|n-1]
	//Ԥ��
	*gx-=gx_cali;
	*gy-=gz_cali;
	*gz-=gz_cali;
	*yaw+=GYRO_TO_DEG(*gz)*T;
	roll_temp=*roll+GYRO_TO_DEG(*gx)*T;
	pitch_temp=*pitch+GYRO_TO_DEG(*gy)*T;
	//��СԤ��MSE
	mPmse_roll[0][0]=mmse_roll[0][0]+(mmse_roll[1][1]+sigma_g)*T*T-(mmse_roll[0][1]+mmse_roll[1][0])*T;
	mPmse_roll[0][1]=mmse_roll[0][1]-mmse_roll[1][1]*T;
	mPmse_roll[1][0]=mmse_roll[1][0]-mmse_roll[1][1]*T;
	mPmse_roll[1][1]=mmse_roll[1][1];
	mPmse_pitch[0][0]=mmse_pitch[0][0]+(mmse_pitch[1][1]+sigma_g)*T*T-(mmse_pitch[0][1]+mmse_pitch[1][0])*T;
	mPmse_pitch[0][1]=mmse_pitch[0][1]-mmse_pitch[1][1]*T;
	mPmse_pitch[1][0]=mmse_pitch[1][0]-mmse_pitch[1][1]*T;
	mPmse_pitch[1][1]=mmse_pitch[1][1];
	//����������
	K_roll[0]=mPmse_roll[0][0]/(mPmse_roll[0][0]+sigma_a);
	K_roll[1]=mPmse_roll[1][0]/(mPmse_roll[0][0]+sigma_a);
	K_pitch[0]=mPmse_pitch[0][0]/(mPmse_pitch[0][0]+sigma_a);
	K_pitch[1]=mPmse_pitch[1][0]/(mPmse_pitch[0][0]+sigma_a);
	//����
	temp=fast_atan2(ay,az)*180/PI-roll_temp;
	*roll=roll_temp+K_roll[0]*temp;
	gx_cali=gx_cali+K_roll[1]*temp;
	temp=fast_atan2(-ax,az)*180/PI-pitch_temp;
	*pitch=pitch_temp+K_pitch[0]*temp;
	gy_cali=gy_cali+K_pitch[1]*temp;
	//��СMSE
	mmse_roll[0][0]=mPmse_roll[0][0]-K_roll[0]*mPmse_roll[0][0];
	mmse_roll[0][1]=mPmse_roll[0][1]-K_roll[0]*mPmse_roll[0][1];
	mmse_roll[1][0]=mPmse_roll[1][0]-K_roll[1]*mPmse_roll[0][0];
	mmse_roll[1][1]=mPmse_roll[1][1]-K_roll[1]*mPmse_roll[0][1];
	mmse_pitch[0][0]=mPmse_pitch[0][0]-K_pitch[0]*mPmse_pitch[0][0];
	mmse_pitch[0][1]=mPmse_pitch[0][1]-K_pitch[0]*mPmse_pitch[0][1];
	mmse_pitch[1][0]=mPmse_pitch[1][0]-K_pitch[1]*mPmse_pitch[0][0];
	mmse_pitch[1][1]=mPmse_pitch[1][1]-K_pitch[1]*mPmse_pitch[0][1];
}