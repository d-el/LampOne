/*!****************************************************************************
 * @file		test_flash.c
 * @author		d_el
 * @version		V1.0
 * @date		May 11, 2022
 * @copyright	License (MIT). Copyright (c) 2021 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
 * @brief
 */
void testFlash(){
	volatile bool start = false;
	while(start == false){
		vTaskDelay(2);
	}
	uart_init(uart1, 115200);
	size_t numberFlash = 0x0437b7df;
	while(1){
		flash_unlock();
		flash_erasePage(&_suser_settings);
		uint32_t data[2] = { numberFlash, ~numberFlash };
		flash_write(&_suser_settings, data, sizeof(data));
		flash_lock();
		if(memcmp(&_suser_settings, data, sizeof(data))){
			break;
		}
		numberFlash++;
		gppin_set(GP_RS485DE);
		char s[16];
		itoa(numberFlash, s, 10);
		strcat(s, "\r\n");
		uart_write(uart1, s, strlen(s));
	}
}

/******************************** END OF FILE ********************************/
