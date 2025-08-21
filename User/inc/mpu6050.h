#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "stm32f10x.h"
#include "string.h"

/************************************************************************************
MPU6050��ؼĴ����궨�壺
************************************************************************************/
#define	SMPLRT_DIV	      	0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIGL			        0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)

#define	GYRO_CONFIG		      0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	      0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define	GYRO_XOUT_H		      0x43  //������ֵ,X���8λ�Ĵ���
#define	GYRO_XOUT_L		      0x44	//������ֵ,X���8λ�Ĵ���
#define	GYRO_YOUT_H		      0x45  //������ֵ,Y���8λ�Ĵ���
#define	GYRO_YOUT_L		      0x46  //������ֵ,Y���8λ�Ĵ���
#define	GYRO_ZOUT_H	      	0x47  //������ֵ,Z���8λ�Ĵ���
#define	GYRO_ZOUT_L		      0x48  //������ֵ,Z���8λ�Ĵ���

#define	ACCEL_XOUT_H        0x3B  //���ٶ�ֵ,X���8λ�Ĵ���
#define	ACCEL_XOUT_L	      0x3C  //���ٶ�ֵ,X���8λ�Ĵ���
#define	ACCEL_YOUT_H	      0x3D  //���ٶ�ֵ,Y���8λ�Ĵ���
#define	ACCEL_YOUT_L      	0x3E  //���ٶ�ֵ,Y���8λ�Ĵ���
#define	ACCEL_ZOUT_H	      0x3F  //���ٶ�ֵ,Z���8λ�Ĵ���
#define	ACCEL_ZOUT_L	      0x40  //���ٶ�ֵ,Z���8λ�Ĵ���

#define	TEMP_OUT_H		      0x41  //�¶�ֵ�߰�λ�Ĵ���
#define	TEMP_OUT_L		      0x42  //�¶�ֵ��8λ�Ĵ���

#define	PWR_MGMT_1	      	0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		        0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_PRODUCT_ID  0x68  //�ź�ͨ����λ�Ĵ���
#define MPU6052C_PRODUCT_ID 0x72
#define MPU_DEVICE_ID	    	0X75	//����ID�Ĵ���
/***********************************************************************************/

/************************************************************************************
MPU6050������ַ��
���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
�����V3.3,��IIC��ַΪ0X69(���������λ).
************************************************************************************/
#define MPU_ADDR				    0X68

#define M_PI 3.1415926f

/* һά���������ݽṹ�� */
struct _1_ekf_filter
{
	float LastP;   //��һ��״̬����������
	float Now_P;   //����һ��״̬����������Ԥ�⵱ǰ״̬����������
	float out;     //��Ԥ�Ȿ״̬����������ʵ����������
	float Kg;      //����������
	float Q;       //ϵͳ���̵�Э����
	float R;	     //������Э����
};

/* MPU6050���ݽṹ�� */
typedef struct{
	//���ٶ�
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	//������
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
