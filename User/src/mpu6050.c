#include "main.h"
#include "math.h"

__align(8) _st_Mpu MPU6050_Data;  
//volatile：每次使用变量时，都必须重新从内存地址读取，不能假设它的值不会变
static volatile int16_t *pMpu = (int16_t *)&MPU6050_Data;
int16_t MpuOffset[6] = {0};

/***************************************************************************************
函数名称  ：kalman_1
函数功能  ：一维卡尔曼滤波器（多维卡尔曼是一维卡尔曼的多维噪声扩展）
函数形参  ：卡尔曼滤波参数，要滤波的数据
函数返回值：无
函数描述  ：
	协方差表示的是两个变量的总体的误差，这与只表示一个变量误差的方差不同。 
		如果两个变量的变化趋势一致，也就是说如果其中一个大于自身的期望值，另外一个也大于自身的期望值，那么两个变量之间的协方差就是正值；
		如果两个变量的变化趋势相反，即其中一个大于自身的期望值，另外一个却小于自身的期望值，那么两个变量之间的协方差就是负值.
****************************************************************************************/
void kalman_1(struct _1_ekf_filter *ekf,float input)            
{
	ekf->Now_P = ekf->LastP + ekf->Q;                    //当前状态的最优评估 = 上一个状态的最优评估 + 系统过程协方差
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);        //卡尔曼增益 = 当前状态的最优评估/（当前状态的最优评估 + 测量协方差）
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);  //最优评估输出 = 最优评估 + 卡尔曼增益 *（输入 - 最优评估输出）
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;              //上一个状态的最优评估 = （1 - 卡尔曼增益）* 当前状态的最优评估
}

/***************************************************************************************
函数名称  ：MPU_Write_Byte
函数功能  ：向mpu6050写一个字节
函数形参  ：寄存器地址，数据
函数返回值：成功返回0，失败返回1
函数描述  ：
****************************************************************************************/
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
	IIC_Start();                                    //IIC起始信号
	IIC_Send_Byte((MPU_ADDR << 1) | 0);             //发送从机地址+写命令(最低位 0 表示写)
	if(IIC_Wait_Ack() == 1)                         //等待应答信号
	{ 
		IIC_Stop();									// 如果没应答，发停止信号
		return 1;									//返回错误
	}
	IIC_Send_Byte(reg);                           	//写寄存器地址
	IIC_Wait_Ack(); 								// 等待应答                                
	IIC_Send_Byte(data);                     		//发送数据 
	IIC_Wait_Ack();									// 等待应答
	IIC_Stop();	                                    //IIC结束信号
	
	return 0;
}

/***************************************************************************************
函数名称  ：MPU_Read_Byte
函数功能  ：mpu6050连续读 
函数形参  ：寄存器地址,数据缓存地址，数据长度
函数返回值：0：读取成功，1：读取失败
函数描述  ：
****************************************************************************************/
u8 MPU_Read_Byte(u8 reg,u8 *data,u8 len)
{
	IIC_Start();                        			//IIC起始信号 
	IIC_Send_Byte((MPU_ADDR << 1) | 0);				//发送器件地址+写命令	
	if(IIC_Wait_Ack() == 1)                         //等待应答信号
	{ 
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	                			//写寄存器地址
	IIC_Wait_Ack();
	IIC_Start();                       				//IIC起始信号 
	IIC_Send_Byte((MPU_ADDR << 1) | 1);				//发送器件地址+读命令
	IIC_Wait_Ack();                                 //等待应答信号
	
	while(len){
		*data = IIC_Read_OneByte();                 //读取数据
		if(len == 1){
				IIC_ACK_Send(1);        			//发送非应答信号
				break;
		}
		else{
				IIC_ACK_Send(0);        			//发送应答信号
				len--;
				data++;
		}	
	}
	IIC_Stop();			                            //IIC结束信号
	return 0;		
}

/***************************************************************************************
函数名称  ：MPU_Get_Temperature
函数功能  ：获取芯片温度
函数形参  ：无
函数返回值：温度值
函数描述  ：
	寄存器中表示负数用的是补码，需要转换成原码才能通过进制转换变成十进制的.
	温度值=-（0xFFFF-直接读出的寄存器十进制值+1）/340+36.53.
****************************************************************************************/
short MPU_Get_Temperature(void)
{
	short raw;
	float temp;
	u8 buff1 = 0;
	u8 buff2 = 0;
	
	MPU_Read_Byte(TEMP_OUT_H,&buff1,1); 		// 读温度高 8 位
	MPU_Read_Byte(TEMP_OUT_L,&buff2,1);			// 读温度低 8 位
	
	raw = ((u16)buff1 << 8) | buff2;  			// 合并成 16 位原始数据
	temp = 36.53 + ((double)raw) / 340;  		// 转换成摄氏度
	
	return temp;								//返回值类型是 short
}

/***************************************************************************************
函数名称  ：MpuGetData
函数功能  ：读取陀螺仪数据加滤波（一维卡尔曼滤波+互补滤波）
函数形参  ：无
函数返回值：无
函数描述  ：
	1.对于夹杂了大量噪音的数据，卡尔曼滤波器的效果无疑是最好的
	2.互补滤波：
			1.加速度计在较长时间的测量值是正确的，而在较短时间内由于信号噪声的存在，而有误差;
			2.陀螺仪在较短时间内则比较准确,而较长时间则会有与漂移而存有误差;
		弥补方法：
			1.短时间内用陀螺仪比较准确，以它为主；
			2.长时间用加速度计比较准确，这时候加大它的比重，这就是互补.
		滤波方法：
			1.加速度计要滤掉高频信号，陀螺仪要滤掉低频信号，互补滤波器就是根据传感器特性不同，
			2.通过不同的滤波器（高通或低通，互补的），然后再相加得到整个频带的信号.
			3.通过高通滤波可抑制低频噪声。
			4.将两者结合，就将陀螺和加表的优点融合起来，得到在高频和低频都较好的信号，
			5.互补滤波需要选择切换的频率点，即高通和低通的频率
****************************************************************************************/
void MpuGetData(void)
{
	uint8_t i;
	uint8_t buffer[12];                            
	
	//读取加速度数据（6字节）
	MPU_Read_Byte(ACCEL_XOUT_H,buffer,6);    //buffer[0-5] 里是 加速度三轴（X、Y、Z，每个16位）
	//读取陀螺仪数据（6字节）
	MPU_Read_Byte(GYRO_XOUT_H,&buffer[6],6);	//buffer[6-11] 里是 陀螺仪三轴（X、Y、Z，每个16位）
	
	for(i=0;i<6;i++)  		//总共12个字节，一组数据占用两个字节，所以总共6组数据
	{	 
		// 拼接高8位和低8位，得到16位原始数据，再减去零偏MpuOffset[i](初始位置)
		/******************************************************************************
		| buffer\[0] | buffer\[1] | buffer\[2] | buffer\[3] | buffer\[4] | buffer\[5] |
		| ---------- | ---------- | ---------- | ---------- | ---------- | ---------- |
		| X 高字节   | X 低字节    | Y 高字节   | Y 低字节   | Z 高字节    | Z 低字节   |
		逐步拆解：
			①i<<1
				i = 0 ,i = 0x0000 → i<<1 = 0x0000 = 0 
				i = 1 ,i = 0x0001 → i<<1 = 0x0010 = 2
				i = 2 ,i = 0x0010 → i<<1 = 0x0100 = 4
				i = 3 ,i = 0x0011 → i<<1 = 0x0110 = 6
				比如 i=0 → 取第0和第1字节（X轴），i=1 → 取第2和第3字节（Y轴），以此类推。
			②buffer[i<<1]
				取第 i 个数据的 高字节。
			③(int16_t)buffer[i<<1] << 8
				高字节左移 8 位，放到 16 位数的高 8 位上。
			举例：
				高字节 = 0x1A
				左移 8 位后 = 0x1A00
			④buffer[(i<<1)+1]
				取第 i 个数据的 低字节。
			⑤|（按位或）
				把高字节和低字节合并起来。
			举例：
				高字节左移后 = 0x1A00
				低字节 = 0x2B
				合并结果 = 0x1A2B
			⑥(int16_t)
				转成 16 位有符号数，MPU6050 输出本来就是 补码形式，所以负数也能正确解析。
		*******************************************************************************/
		//每个通道（加速度 X、Y、Z，陀螺仪 X、Y、Z）都会有零点偏差。
		//MpuOffset[i] 存的是这些偏差值。
		//这样得到的数据就是 校准后的传感器读数。
		//最终结果放到 pMpu[i]，也就是 MPU6050_Data 的第 i 个成员。
		pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];	 
		
		
		/*****************************************************************
		① if(i < 3)
			表示处理 前三个数据（一般是 加速度 X/Y/Z）。
			用 一维卡尔曼滤波器 kalman_1() 来消除噪声。
			ekf[i] 是三个卡尔曼滤波器的实例，分别对应加速度 X、Y、Z。
			pMpu[i] 最后保存卡尔曼滤波后的结果。
		********************************************************************/
		if(i < 3)
		{
			{
				static struct _1_ekf_filter ekf[3] = {		
							{0.02,0,0,0,0.001,0.543},
							{0.02,0,0,0,0.001,0.543},
							{0.02,0,0,0,0.001,0.543}
				};					
				kalman_1(&ekf[i],(float)pMpu[i]);  // 一维卡尔曼滤波
				pMpu[i] = (int16_t)ekf[i].out;	// 输出滤波后的数据
			}
		}
		/*****************************************************************
		② if(i > 2)
			表示处理 后三个数据（一般是 陀螺仪 X/Y/Z）。
			这里用的是 一阶低通滤波（指数滑动平均法）：
						y[k]=(1-α)*y[k-1]+α*x[k]
			tBuff[k] → 保存上一次的滤波结果。
			pMpu[i] → 当前的原始数据。
			factor = 0.15 → 滤波系数，越小数据越平滑，但延迟越大。
		********************************************************************/		
		if(i > 2)
		{	
			uint8_t k = i - 3;
			const float factor = 0.15f;          //滤波因素			
			static float tBuff[3];		

			pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;                
		}
		/*****************************************************************
		加速度数据 (i = 0,1,2)：用 一维卡尔曼滤波，对动态噪声的抑制更强，适合姿态解算。
		陀螺仪数据 (i = 3,4,5)：用 低通滤波（指数平均），减少高频噪声。
		********************************************************************/			
	}
}

/***************************************************************************************
函数名称  ：Mpu_GetOffset
函数功能  ：陀螺仪校准
函数形参  ：无
函数返回值：无
函数描述  ：
	开机静止放置飞行器求零偏,将三轴陀螺仪数据校准到设置范围内，每次获取到的MPU数据减去这个偏移量.
****************************************************************************************/
void GyroGetOffset(void) 
{
	int32_t buffer[6]={0};
	int16_t i;  
	uint8_t k=30;
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;	
	/*wait for calm down*/
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];	
	memset(MpuOffset,0,12);
	MpuOffset[2] = 4096;   //set offset from the 8192  
	
	while(k--)
	{
		do
		{
			Delay_ms(10);
			MpuGetData();
			for(i=0;i<3;i++)
			{
				ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
				LastGyro[i] = pMpu[i+3];	
			}			
		}while ((ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)
					||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
					||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
						);
	}	
	
	for(i=0;i<356;i++)
	{		
		MpuGetData();
		
		if(100 <= i)
		{
			uint8_t k;
			for(k=0;k<6;k++)
			{
				buffer[k] += pMpu[k];
			}
		}
	}

	for(i=0;i<6;i++)
	{
		MpuOffset[i] = buffer[i]>>8;
	}
}

/***************************************************************************************
函数名称  ：MPU_Init
函数功能  ：mpu6050初始化
函数形参  ：无
函数返回值：返回0陀螺仪初始化成功，其他为未检测到陀螺仪
函数描述  ：
****************************************************************************************/
u8 MPU_Init(void)
{ 
	u8 res;
	do{
		res = MPU_Write_Byte(PWR_MGMT_1, 0x80);   //复位MPU6050
		Delay_ms(20);
		res = MPU_Write_Byte(PWR_MGMT_1, 0x00);   //唤醒MPU6050
		res += MPU_Write_Byte(SMPLRT_DIV, 0x02);  //陀螺仪采样率，0x00(500Hz)
		res += MPU_Write_Byte(PWR_MGMT_1, 0x03);  //设置设备时钟源，PLL Z 轴为参考
		res += MPU_Write_Byte(CONFIGL, 0x03);     //低通滤波频率，陀螺仪带宽：0x03(42Hz)
		res += MPU_Write_Byte(GYRO_CONFIG, 0x18); //+-2000deg/s，不自检(灵敏度为 16.4 LSB/°/s)
		res += MPU_Write_Byte(ACCEL_CONFIG, 0x10);//+-8G，不自检
	}while(res != 0);                             	  //res == 0，说明设置成功

	MPU_Read_Byte(MPU_DEVICE_ID,&res,1); 			  //读取器件ID寄存器的器件地址
	if(res != MPU_ADDR)                           	  //判断MPU是否在位
		return 1;      			
	else 
		/* -----------校准陀螺仪 - 静止求零偏----------- */
		GyroGetOffset();
	return 0;
}


float pitch = 0, roll = 0;
void MPU_Update_Attitude(void) {
	
    MpuGetData();  // 这里不要再对 accZ 减去“1g/2g”的固定偏置

    // 量程换算：±8g -> 4096 LSB/g；±2000 dps -> 16.4 LSB/(deg/s)
	float ax = MPU6050_Data.accX / 4096.0f;
    float ay = MPU6050_Data.accY / 4096.0f;
    float az = MPU6050_Data.accZ / 4096.0f;

    float gx = MPU6050_Data.gyroX / 16.4f;
    float gy = MPU6050_Data.gyroY / 16.4f;
    float gz = MPU6050_Data.gyroZ / 16.4f; 
	 
	//有bug跟下面数据解析参数不匹配，先确定好安装方向
	// （可选）如果你确认 X/Y 反了，就在此交换：根据模块安装方向以及设备运动方向确定pitch和roll
    //float t = ax; ax = ay; ay = t;
	
	
	// 归一化（很关键，能兜住比例误差）
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-6f) { ax/=norm; ay/=norm; az/=norm; }	

	
	// 仅用加速度得到的姿态（避免万向锁的常用写法）
	float pitch_acc = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
	float roll_acc  = atan2f(-ay, az) * 180.0f / M_PI;

	
	// dt（请用真实时间戳计算；这里示例 10ms）
    const float dt = 0.01f;   //10ms
    const float alpha = 0.90f;
	
	
	// 互补滤波（约定：绕 X 的角称 roll，绕 Y 的角称 pitch）
	pitch = alpha * (pitch + (-gy) * dt) + (1.0f - alpha) * pitch_acc;
	roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * roll_acc;
}

