#include "main.h"

/***********************************************************************************
内部FLASH：
	内部 FLASH 包含主存储器、系统存储器、 OTP 区域以及选项字节区域。
	主存储器区：域它是存储用户应用程序的空间，
						 芯片型号说明中的 1M FLASH、 2M FLASH 都是指这个区域的大小，
	           与其它 FLASH 一样，在写入数据前，要先按扇区擦除。
	系统存储区：系统存储区是用户不能访问的区域，它在芯片出厂时已经固化了启动代码，
						 它负责实现串口、 USB 以及 CAN 等 ISP 烧录功能。
	OTP 区域：OTP(One Time Program)，指的是只能写入一次的存储区域，
						 容量为 512 字节，写入后数据就无法再更改， OTP 常用于存储应用程序的加密密钥。
	选项字节：选项字节用于配置 FLASH 的读写保护、电源管理中的 BOR 级别、软件/硬件看门狗等功能，
					  这部分共 32 字节。可以通过修改 FLASH 的选项控制寄存器修改。
***********************************************************************************/

typedef enum {FAILED = 0, PASSED = !FAILED} Status;

uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t First_Page = 0x00;                        //起始页

__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO Status MemoryProgramStatus = PASSED;


/*****************************************************************************
函数名称：FLASH_write
函数功能：写FLASH
函数形参：数据，数据长度,起始地址
函数返回值：无
函数描述：
	1. 解锁 （固定的KEY值）
	2. 数据操作位数 （大部分应用场合都是用 32 位的宽度）
	3. 擦除扇区 （STM32 提供了扇区擦除指令和整个FLASH 擦除(批量擦除)的指令，批量擦除指令仅针对主存储区）
*****************************************************************************/
void FLASH_write(int16_t *data,uint8_t len)
{
	FLASH_Unlock();//先解锁
	
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); //清除相应的标志位
	First_Page = (FLASH_End_Addr - FLASH_Start_Addr + 1) / FLASH_Page_Size;   //计算出起始页
	
	/* 使用前先擦除 */
	for(EraseCounter = 0; (EraseCounter < First_Page) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		/*检查 FLASH_SR 寄存器中的“忙碌寄存器位 BSY”，以确认当前未执行任何 */
		if (FLASH_ErasePage(FLASH_Start_Addr + (FLASH_Page_Size * EraseCounter)) != FLASH_COMPLETE)
		{
			while (1);
		}
	}
	
	/* 写入FLASH */
	Address = FLASH_Start_Addr;     
	while (len--)
	{
		/*检查 FLASH_SR 中的 BSY 位，以确认当前未执行任何其它的内部 Flash 操作 */
		if(FLASH_ProgramHalfWord(Address, *data) == FLASH_COMPLETE)
		{
			data++;
			Address = Address + 2;
		}
		else
		{ 
			while (1);
		}
	}
	
	/* 上锁 */
	FLASH_Lock();
}	

/*****************************************************************************
函数名称：FLASH_read
函数功能：读FLASH
函数形参：数据，数据长度
函数返回值：无
函数描述：
*****************************************************************************/
void FLASH_read(uint32_t *data,uint8_t len)
{
	Address = FLASH_Start_Addr;
	
	while (len--)
	{
		*data = *(__IO uint32_t *)Address;
		data++;
		Address = Address + 4;
	}
}
