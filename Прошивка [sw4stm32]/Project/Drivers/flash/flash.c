#include "stm32f0xx_hal.h"

void flash_unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;								// Запись в регистр первого ключа
	FLASH->KEYR = FLASH_KEY2;								// Запись в регистр второго ключа
}

void flash_lock()
{
	FLASH->CR |= FLASH_CR_LOCK_Msk;					// Установка бита блокировки FLASH
}

uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);				// Проверка бита готовности FLASH
	HAL_Delay(10);
}

uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);							// Возвращает значение слова в FLASH по указанному адресу
}

void flash_erase_page(uint32_t address)
{
	FLASH->CR|= FLASH_CR_PER_Msk;						// Установка бита разрешения очистки страницы
	FLASH->AR = address;												// Запись адреса страницы
	FLASH->CR|= FLASH_CR_STRT_Msk;						// Запуск операции стирания
	while(!flash_ready());												// Ожидание завершения процесса стирания
	FLASH->CR &=~ FLASH_CR_PER_Msk;				// Сброс бита разрешения очистки страницы
}

//Запись ячейки FLASH
void flash_write(uint32_t address,uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG_Msk;						// Установка бита разрешения программирования
	while(!flash_ready());												// Ожидание готовности FLASH
	
	*(__IO uint16_t*)address = (uint16_t)data;			// Запись младшего полуслова по указанному адресу
	while(!flash_ready());												// Ожидание готовности FLASH
	
	address+=2;																// Смещение адреса на два байта (одно полуслово)
	data>>=16;																// Смещение данных для записи старшего полуслова
	*(__IO uint16_t*)address = (uint16_t)data;			// Запись старшего полуслова по указанному адресу
	while(!flash_ready());												// Ожидание готовности FLASH
	
	FLASH->CR &=~ FLASH_CR_PG_Msk;					// Сброс бита разрешения программирования
}

void flash_write_float(uint32_t address, uint32_t data)
{
	FLASH->CR |= FLASH_CR_PG_Msk;						// Установка бита разрешения программирования
	while(!flash_ready());												// Ожидание готовности FLASH

	*(__IO uint16_t*)address = data;								// Запись младшего полуслова по указанному адресу
	while(!flash_ready());												// Ожидание готовности FLASH

	address+=2;																// Смещение адреса на два байта (одно полуслово)
	data>>=16;																// Смещение данных для записи старшего полуслова
	*(__IO uint16_t*)address = data;								// Запись старшего полуслова по указанному адресу
	while(!flash_ready());												// Ожидание готовности FLASH

	FLASH->CR &=~ FLASH_CR_PG_Msk;					// Сброс бита разрешения программирования
}

float flash_read_float(uint32_t address)
{
	return (*(__IO float*) address);								// Возвращает значение слова в FLASH по указанному адресу
}
