#define SAVEKEY 0xAABBCCDD											// Ключевое (проверочное) слово

void flash_unlock(void);														// Разблокировка FLASH
void flash_lock();																		// Блокировка FLASH
uint8_t flash_ready(void);														// Проверка готовности FLASH
uint32_t flash_read(uint32_t address);									// Чтение FLASH
void flash_erase_page(uint32_t address);							// Стирание страницы FLASH
void flash_write(uint32_t address,uint32_t data);				// Запись слова во FLASH
void flash_write_float(uint32_t address, float data);		// Запись числа с плавающей точкой
float flash_read_float(uint32_t address);							// Чтение числа с плавающей точкой
