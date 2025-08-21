#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "stm32f10x.h"
#include "string.h"

/************************************************************************************
MPU6050相关寄存器宏定义：
************************************************************************************/
#define	SMPLRT_DIV	      	0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			        0x1A	//低通滤波频率，典型值：0x06(5Hz)

#define	GYRO_CONFIG		      0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	      0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	GYRO_XOUT_H		      0x43  //陀螺仪值,X轴高8位寄存器
#define	GYRO_XOUT_L		      0x44	//陀螺仪值,X轴低8位寄存器
#define	GYRO_YOUT_H		      0x45  //陀螺仪值,Y轴高8位寄存器
#define	GYRO_YOUT_L		      0x46  //陀螺仪值,Y轴低8位寄存器
#define	GYRO_ZOUT_H	      	0x47  //陀螺仪值,Z轴高8位寄存器
#define	GYRO_ZOUT_L		      0x48  //陀螺仪值,Z轴低8位寄存器

#define	ACCEL_XOUT_H        0x3B  //加速度值,X轴高8位寄存器
#define	ACCEL_XOUT_L	      0x3C  //加速度值,X轴低8位寄存器
#define	ACCEL_YOUT_H	      0x3D  //加速度值,Y轴高8位寄存器
#define	ACCEL_YOUT_L      	0x3E  //加速度值,Y轴低8位寄存器
#define	ACCEL_ZOUT_H	      0x3F  //加速度值,Z轴高8位寄存器
#define	ACCEL_ZOUT_L	      0x40  //加速度值,Z轴低8位寄存器

#define	TEMP_OUT_H		      0x41  //温度值高八位寄存器
#define	TEMP_OUT_L		      0x42  //温度值低8位寄存器

#define	PWR_MGMT_1	      	0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		        0x75	//IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID  0x68  //信号通道复位寄存器
#define MPU6052C_PRODUCT_ID 0x72
#define MPU_DEVICE_ID	    	0X75	//器件ID寄存器
/***********************************************************************************/

/************************************************************************************
MPU6050器件地址：
如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
如果接V3.3,则IIC地址为0X69(不包含最低位).
************************************************************************************/
#define MPU_ADDR				    0X68

#define M_PI 3.1415926f

/* 一维卡尔曼数据结构体 */
struct _1_ekf_filter
{
	float LastP;   //上一个状态的最优评估
	float Now_P;   //由上一个状态的最优评估预测当前状态的最优评估
	float out;     //由预测本状态的评估具体实现最优评估
	float Kg;      //卡尔曼增益
	float Q;       //系统过程的协方差
	float R;	     //测量的协方差
};

/* MPU6050数据结构体 */
typedef struct{
	//加速度
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	//陀螺仪
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;


extern _st_Mpu MPU6050_Data; 
extern float pitch, roll;

u8 MPU_Write_Byte(u8 reg,u8 data);
u8 MPU_Read_Byte(u8 reg,u8 *data,u8 len);
short MPU_Get_Temperature(void);
u8 MPU_Init(void);
void MpuGetData(void);
void GyroGetOffset(void);
void MPU_Update_Attitude(void);
#endif
