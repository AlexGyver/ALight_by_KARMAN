#include "stm32f0xx_hal.h"

void flash_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;								// ������ � ������� ������� �����
	FLASH->KEYR = FLASH_KEY2;								// ������ � ������� ������� �����
}

void flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK_Msk;					// ��������� ���� ���������� FLASH
}

uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);				// �������� ���� ���������� FLASH
	HAL_Delay(10);
}

uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);							// ���������� �������� ����� � FLASH �� ���������� ������
}

void flash_erase_page(uint32_t address)
{
	FLASH->CR|= FLASH_CR_PER_Msk;						// ��������� ���� ���������� ������� ��������
	FLASH->AR = address;												// ������ ������ ��������
	FLASH->CR|= FLASH_CR_STRT_Msk;						// ������ �������� ��������
	while(!flash_ready());												// �������� ���������� �������� ��������
	FLASH->CR &=~ FLASH_CR_PER_Msk;				// ����� ���� ���������� ������� ��������
}

//������ ������ FLASH
void flash_write(uint32_t address,uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG_Msk;						// ��������� ���� ���������� ����������������
	while(!flash_ready());												// �������� ���������� FLASH
	
	*(__IO uint16_t*)address = (uint16_t)data;			// ������ �������� ��������� �� ���������� ������
	while(!flash_ready());												// �������� ���������� FLASH
	
	address+=2;																// �������� ������ �� ��� ����� (���� ���������)
	data>>=16;																// �������� ������ ��� ������ �������� ���������
	*(__IO uint16_t*)address = (uint16_t)data;			// ������ �������� ��������� �� ���������� ������
	while(!flash_ready());												// �������� ���������� FLASH
	
	FLASH->CR &=~ FLASH_CR_PG_Msk;					// ����� ���� ���������� ����������������
}

void flash_write_float(uint32_t address, uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG_Msk;						// ��������� ���� ���������� ����������������
	while(!flash_ready());												// �������� ���������� FLASH

	*(__IO uint16_t*)address = data;								// ������ �������� ��������� �� ���������� ������
	while(!flash_ready());												// �������� ���������� FLASH

	address+=2;																// �������� ������ �� ��� ����� (���� ���������)
	data>>=16;																// �������� ������ ��� ������ �������� ���������
	*(__IO uint16_t*)address = data;								// ������ �������� ��������� �� ���������� ������
	while(!flash_ready());												// �������� ���������� FLASH

	FLASH->CR &=~ FLASH_CR_PG_Msk;					// ����� ���� ���������� ����������������
}

float flash_read_float(uint32_t address)
{
	return (*(__IO float*) address);								// ���������� �������� ����� � FLASH �� ���������� ������
}
