#define SAVEKEY 0xAABBCCDD											// �������� (�����������) �����

void flash_unlock(void);														// ������������� FLASH
void flash_lock();																		// ���������� FLASH
uint8_t flash_ready(void);														// �������� ���������� FLASH
uint32_t flash_read(uint32_t address);									// ������ FLASH
void flash_erase_page(uint32_t address);							// �������� �������� FLASH
void flash_write(uint32_t address,uint32_t data);				// ������ ����� �� FLASH
void flash_write_float(uint32_t address, float data);		// ������ ����� � ��������� ������
float flash_read_float(uint32_t address);							// ������ ����� � ��������� ������
