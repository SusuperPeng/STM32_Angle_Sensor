#include "main.h"


int main(void)
{
	/* �����ж����ȼ����� -- ѡ����2 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* Ӳ����ʼ�� */
	TIM_4_Init();        	// ͨ�ö�ʱ��4��ʼ�� - ������ʱ
	USART1_Init();        	// USART1��ʼ�� - ������λ��ͨ��
	IIC_GPIO_Init();        // IIC��ʼ��
	if(MPU_Init()) 			// MPU6050��ʼ��
	printf("mpu6050 error��\r\n");

	/* ������ʾ��ʼ���ɹ� */
	printf("��ʼ���ɹ���\r\n");
	
	while(1)
	{
		
		MpuGetData();			
		MPU_Update_Attitude();
		printf("%.2f,%.2f\r\n", pitch, roll);  //pitch��X�ᣬroll��Y��
	}
}
