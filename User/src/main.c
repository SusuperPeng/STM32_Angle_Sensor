#include "main.h"


int main(void)
{
	/* 配置中断优先级分组 -- 选择组2 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* 硬件初始化 */
	TIM_4_Init();        	// 通用定时器4初始化 - 用于延时
	USART1_Init();        	// USART1初始化 - 用于上位机通信
	IIC_GPIO_Init();        // IIC初始化
	if(MPU_Init()) 			// MPU6050初始化
	printf("mpu6050 error！\r\n");

	/* 串口提示初始化成功 */
	printf("初始化成功！\r\n");
	
	while(1)
	{
		
		MpuGetData();			
		MPU_Update_Attitude();
		printf("%.2f,%.2f\r\n", pitch, roll);  //pitch绕X轴，roll绕Y轴
	}
}
