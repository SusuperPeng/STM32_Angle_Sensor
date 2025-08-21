#include "main.h"

/***********************************************************************************
�ڲ�FLASH��
	�ڲ� FLASH �������洢����ϵͳ�洢���� OTP �����Լ�ѡ���ֽ�����
	���洢�����������Ǵ洢�û�Ӧ�ó���Ŀռ䣬
						 оƬ�ͺ�˵���е� 1M FLASH�� 2M FLASH ����ָ�������Ĵ�С��
	           ������ FLASH һ������д������ǰ��Ҫ�Ȱ�����������
	ϵͳ�洢����ϵͳ�洢�����û����ܷ��ʵ���������оƬ����ʱ�Ѿ��̻����������룬
						 ������ʵ�ִ��ڡ� USB �Լ� CAN �� ISP ��¼���ܡ�
	OTP ����OTP(One Time Program)��ָ����ֻ��д��һ�εĴ洢����
						 ����Ϊ 512 �ֽڣ�д������ݾ��޷��ٸ��ģ� OTP �����ڴ洢Ӧ�ó���ļ�����Կ��
	ѡ���ֽڣ�ѡ���ֽ��������� FLASH �Ķ�д��������Դ�����е� BOR �������/Ӳ�����Ź��ȹ��ܣ�
					  �ⲿ�ֹ� 32 �ֽڡ�����ͨ���޸� FLASH ��ѡ����ƼĴ����޸ġ�
***********************************************************************************/

typedef enum {FAILED = 0, PASSED = !FAILED} Status;

uint32_t EraseCounter = 0x00, Address = 0x00;
uint32_t Data = 0x3210ABCD;
uint32_t First_Page = 0x00;                        //��ʼҳ

__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO Status MemoryProgramStatus = PASSED;


/*****************************************************************************
�������ƣ�FLASH_write
�������ܣ�дFLASH
�����βΣ����ݣ����ݳ���,��ʼ��ַ
��������ֵ����
����������
	1. ���� ���̶���KEYֵ��
	2. ���ݲ���λ�� ���󲿷�Ӧ�ó��϶����� 32 λ�Ŀ�ȣ�
	3. �������� ��STM32 �ṩ����������ָ�������FLASH ����(��������)��ָ���������ָ���������洢����
*****************************************************************************/
void FLASH_write(int16_t *data,uint8_t len)
{
	FLASH_Unlock();//�Ƚ���
	
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); //�����Ӧ�ı�־λ
	First_Page = (FLASH_End_Addr - FLASH_Start_Addr + 1) / FLASH_Page_Size;   //�������ʼҳ
	
	/* ʹ��ǰ�Ȳ��� */
	for(EraseCounter = 0; (EraseCounter < First_Page) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		/*��� FLASH_SR �Ĵ����еġ�æµ�Ĵ���λ BSY������ȷ�ϵ�ǰδִ���κ� */
		if (FLASH_ErasePage(FLASH_Start_Addr + (FLASH_Page_Size * EraseCounter)) != FLASH_COMPLETE)
		{
			while (1);
		}
	}
	
	/* д��FLASH */
	Address = FLASH_Start_Addr;     
	while (len--)
	{
		/*��� FLASH_SR �е� BSY λ����ȷ�ϵ�ǰδִ���κ��������ڲ� Flash ���� */
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
	
	/* ���� */
	FLASH_Lock();
}	

/*****************************************************************************
�������ƣ�FLASH_read
�������ܣ���FLASH
�����βΣ����ݣ����ݳ���
��������ֵ����
����������
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
