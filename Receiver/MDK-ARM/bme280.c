#include "BME280.h"
//------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

char bme_str[50];
bme280_cal_t cal_data;
int32_t temper_int;

//------------------------------------------------
void bme280_init(void)
{
  uint8_t value = 0;
  uint32_t value32 = 0;
	value = bme280_read_reg(BME280_REG_ID);
	
	sprintf(bme_str, "ID: 0x%02X\r\n", value);
	HAL_UART_Transmit(&huart1,(uint8_t*)bme_str, strlen(bme_str), 0x1000);
	bme280_wr_reg(BME280_REG_SOFTRESET,BME280_SOFTRESET_VALUE);
	while (bme280_read_status() & BME280_STATUS_IM_UPDATE);

	bme280_set_standby(BME280_STBY_1000);
	bme280_set_filter(BME280_FILTER_OFF);
	bme280_set_oversamp_tempreture(BME280_OSRS_T_x1);
	bme280_set_oversamp_pressure(BME280_OSRS_P_x1);
	bme280_set_oversamp_humidity(BME280_OSRS_H_x1);
	bme280_read_coeffs();
	#if DEBUGENABLED == 1
	value32 = bme280_read_reg(BME280_REG_CTRL_MEAS);
	value32 |= bme280_read_reg(BME280_REG_CTRL_HUM) << 8;
	sprintf(bme_str, "Measurements status: %04X\r\n", value32);
	HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
	sprintf(bme_str, "Temperature: %s\r\nPressure: %s\r\nHumidity: %s\r\n",
					(value32 & BME280_OSRS_T_MSK) ? "ON" : "OFF",
					(value32 & BME280_OSRS_P_MSK) ? "ON" : "OFF",
					((value32 >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF");
	HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);

	#endif
	bme280_set_mode(BME280_MODE_NORMAL);
}

static void i2c_wr_byte(uint16_t addr, uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, addr, (uint16_t)reg, 
														 I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
}

static uint8_t i2c_read_byte(uint16_t addr, uint8_t reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, addr, reg, 
														I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  return value;
}

static void i2c_read_data2b(uint16_t addr, uint8_t reg, uint16_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, addr, reg,
														I2C_MEMADD_SIZE_8BIT, (uint8_t*)value, 2, 0x10000); 
}

static void i2c_read_data3b(uint16_t addr, uint8_t reg, uint32_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, addr, reg,
														I2C_MEMADD_SIZE_8BIT, (uint8_t*)value, 3, 0x10000);
}

void bme280_wr_reg(uint8_t reg, uint8_t value)
{
  i2c_wr_byte(BME280_ADDR, reg, value);
}

uint8_t bme280_read_reg(uint8_t reg)
{
  uint8_t res = i2c_read_byte(BME280_ADDR,reg);
  return res;
}

uint8_t bme280_read_status(void)
{
  uint8_t res = bme280_read_reg(BME280_REGISTER_STATUS) & 0x09;
  return res;
}

void bme280_read_reg_u2b(uint8_t reg, uint16_t *value)
{
  i2c_read_data2b(BME280_ADDR,reg,value);
}

void bme280_read_reg_s2b(uint8_t reg, int16_t *value)
{
  i2c_read_data2b(BME280_ADDR,reg,(uint16_t*)value);
}

void bme280_read_reg_be_s2b(uint8_t reg, int16_t *value)
{
  i2c_read_data2b(BME280_ADDR,reg,(uint16_t*)value);
  *(uint16_t *) value = be2btoword(*(uint16_t *) value);
}

void bme280_read_reg_u3b(uint8_t reg, uint32_t *value)
{
  i2c_read_data3b(BME280_ADDR, reg, value);
  *(uint32_t *) value &= 0x00FFFFFF;
}

void bme280_read_reg_be_u3b(uint8_t reg, uint32_t *value)
{
  i2c_read_data3b(BME280_ADDR,reg,value);
  *(uint32_t *) value = be3btoword(*(uint32_t *) value) & 0x00FFFFFF;
}

void bme280_read_coeffs(void)
{	
  bme280_read_reg_u2b(BME280_REGISTER_DIG_T1,&cal_data.dig_T1);
  sprintf(bme_str, "DIG_T1: %u\r\n", cal_data.dig_T1);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_T2,&cal_data.dig_T2);
  sprintf(bme_str, "DIG_T2: %d\r\n", cal_data.dig_T2);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_T3,&cal_data.dig_T3);
  sprintf(bme_str, "DIG_T3: %d\r\n", cal_data.dig_T3);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_u2b(BME280_REGISTER_DIG_P1,&cal_data.dig_P1);
  sprintf(bme_str, "DIG_P1: %u\r\n", cal_data.dig_P1);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P2,&cal_data.dig_P2);
  sprintf(bme_str, "DIG_P2: %d\r\n", cal_data.dig_P2);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P3,&cal_data.dig_P3);
  sprintf(bme_str, "DIG_P3: %d\r\n", cal_data.dig_P3);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P4,&cal_data.dig_P4);
  sprintf(bme_str, "DIG_P4: %d\r\n", cal_data.dig_P4);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P5,&cal_data.dig_P5);
  sprintf(bme_str, "DIG_P5: %d\r\n", cal_data.dig_P5);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P6,&cal_data.dig_P6);
  sprintf(bme_str, "DIG_P6: %d\r\n", cal_data.dig_P6);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P7,&cal_data.dig_P7);
  sprintf(bme_str, "DIG_P7: %d\r\n", cal_data.dig_P7);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P8,&cal_data.dig_P8);
  sprintf(bme_str, "DIG_P8: %d\r\n", cal_data.dig_P8);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_P9,&cal_data.dig_P9);
  sprintf(bme_str, "DIG_P9: %d\r\n", cal_data.dig_P9);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  cal_data.dig_H1 = bme280_read_reg(BME280_REGISTER_DIG_H1);
  sprintf(bme_str, "DIG_H1: %d\r\n", cal_data.dig_H1);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  bme280_read_reg_s2b(BME280_REGISTER_DIG_H2,&cal_data.dig_H2);
  sprintf(bme_str, "DIG_H2: %d\r\n", cal_data.dig_H2);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  cal_data.dig_H3 = bme280_read_reg(BME280_REGISTER_DIG_H3);
  sprintf(bme_str, "DIG_H3: %d\r\n", cal_data.dig_H3);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  cal_data.dig_H4 = (bme280_read_reg(BME280_REGISTER_DIG_H4) << 4) | (bme280_read_reg(BME280_REGISTER_DIG_H4+1) & 0xF);	
  sprintf(bme_str, "DIG_H4: %d\r\n", cal_data.dig_H4);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  cal_data.dig_H5 = (bme280_read_reg(BME280_REGISTER_DIG_H5+1) << 4) | (bme280_read_reg(BME280_REGISTER_DIG_H5) >> 4);
  sprintf(bme_str, "DIG_H5: %d\r\n", cal_data.dig_H5);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
  cal_data.dig_H6 = (int8_t)bme280_read_reg(BME280_REGISTER_DIG_H6);
  sprintf(bme_str, "DIG_H6: %d\r\n", cal_data.dig_H3);
  HAL_UART_Transmit(&huart1,(uint8_t*)bme_str,strlen(bme_str),0x1000);
}

void bme280_set_standby(uint8_t tsb)
{
  uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;
  reg |= tsb & BME280_STBY_MSK;
  bme280_wr_reg(BME280_REG_CONFIG, reg);
}

void bme280_set_filter(uint8_t filter)
{
	uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;
  reg |= filter & BME280_FILTER_MSK;
  bme280_wr_reg(BME280_REG_CONFIG, reg);
}

void bme280_set_oversamp_tempreture(uint8_t osrs)
{
  uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;
  reg |= osrs & BME280_OSRS_T_MSK;
  bme280_wr_reg(BME280_REG_CTRL_MEAS,reg);
}

void bme280_set_oversamp_pressure(uint8_t osrs)
{
  uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;
  reg |= osrs & BME280_OSRS_P_MSK;
  bme280_wr_reg(BME280_REG_CTRL_MEAS,reg);
}

void bme280_set_oversamp_humidity(uint8_t osrs)
{
  uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;
  reg |= osrs & BME280_OSRS_H_MSK;
  bme280_wr_reg(BME280_REG_CTRL_HUM,reg);
  //The 'ctrl_hum' register needs to be written
  //after changing 'ctrl_hum' for the changes to become effective.
  reg = bme280_read_reg(BME280_REG_CTRL_MEAS);
  bme280_wr_reg(BME280_REG_CTRL_MEAS,reg);
}

void bme280_set_mode(uint8_t mode)
{
  uint8_t reg;
  reg = bme280_read_reg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;
  reg |= mode & BME280_MODE_MSK;
  bme280_wr_reg(BME280_REG_CTRL_MEAS,reg);
}

float bme280_read_temperature(void)
{
  float temper_float = 0.0f;
	uint32_t temper_raw;
	int32_t temp1, 
					temp2;
	
	bme280_read_reg_be_u3b(BME280_REGISTER_TEMPDATA, &temper_raw);
	temper_raw >>= 4;
	temp1 = ((((temper_raw>>3) - ((int32_t)cal_data.dig_T1 <<1))) *
				 ((int32_t)cal_data.dig_T2)) >> 11;
	temp2 = (((((temper_raw>>4) - ((int32_t)cal_data.dig_T1)) *
				 ((temper_raw>>4) - ((int32_t)cal_data.dig_T1))) >> 12) *
				 ((int32_t)cal_data.dig_T3)) >> 14;
	temper_int = temp1 + temp2;
	temper_float = ((temper_int * 5 + 128) >> 8);
	temper_float /= 100.0f;
	
  return temper_float;
}

float bme280_read_pressure(void)
{
  float press_float = 0.0f;
	uint32_t press_raw, pres_int;
	int64_t temp1, temp2, p;
	
//	bme280_read_temperature(); 			// must be done first
	bme280_read_reg_be_u3b(BME280_REGISTER_PRESSUREDATA, &press_raw);
	press_raw >>= 4;
	temp1 = ((int64_t) temper_int) - 128000;
	temp2 = temp1 * temp1 * (int64_t)cal_data.dig_P6;
	temp2 = temp2 + ((temp1 * (int64_t)cal_data.dig_P5) << 17);
	temp2 = temp2 + ((int64_t)cal_data.dig_P4 << 35);
	temp1 = ((temp1 * temp1 * (int64_t)cal_data.dig_P3) >> 8) + ((temp1 * (int64_t)cal_data.dig_P2) << 12);
	temp1 = (((((int64_t)1) << 47) + temp1)) * ((int64_t)cal_data.dig_P1) >> 33;
	if (temp1 == 0)
		return 0; 									// avoid exception caused by division by zero
	p = 1048576 - press_raw;
	p = (((p << 31) - temp2) * 3125) / temp1;
	temp1 = (((int64_t)cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	temp2 = (((int64_t)cal_data.dig_P8) * p) >> 19;
	p = ((p + temp1 + temp2) >> 8) + ((int64_t)cal_data.dig_P7 << 4);
	pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
	press_float = pres_int / 100.0f;
	
	
  return press_float;
}

float bme280_read_humidity(void)
{
  float hum_float = 0.0f;
	int16_t hum_raw;
	int32_t hum_raw_sign, 
					v_x1_u32r;
	
	
//	bme280_read_temperature(); // must be done first
	bme280_read_reg_be_s2b(BME280_REGISTER_HUMIDDATA, &hum_raw);
	hum_raw_sign = ((int32_t)hum_raw)&0x0000FFFF;
	
	v_x1_u32r = (temper_int - ((int32_t)76800));
	v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t)cal_data.dig_H4) << 20) -
	(((int32_t)cal_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	(((((((v_x1_u32r * ((int32_t)cal_data.dig_H6)) >> 10) *
	(((v_x1_u32r * ((int32_t)cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
	((int32_t)2097152)) * ((int32_t)cal_data.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((int32_t)cal_data.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	hum_float = (v_x1_u32r>>12);
	hum_float /= 1024.0f;

	
  return hum_float;
}

void bme280_get_meas(bmp_meas_t *bmp)
{
	uint32_t temper_raw;
	int32_t temp_t1, 
					temp_t2;
	
	uint32_t press_raw, pres_int;
	int64_t temp_p1, temp_p2, p;

	int16_t hum_raw;
	int32_t hum_raw_sign, 
					v_x1_u32r;
	
	bme280_read_reg_be_u3b(BME280_REGISTER_TEMPDATA, &temper_raw);
	bme280_read_reg_be_u3b(BME280_REGISTER_PRESSUREDATA, &press_raw);
	bme280_read_reg_be_s2b(BME280_REGISTER_HUMIDDATA, &hum_raw);
//-------------------Temperature-------------------------------------	
	temper_raw >>= 4;
	temp_t1 = ((((temper_raw>>3) - ((int32_t)cal_data.dig_T1 <<1))) *
				 ((int32_t)cal_data.dig_T2)) >> 11;
	temp_t2 = (((((temper_raw>>4) - ((int32_t)cal_data.dig_T1)) *
				 ((temper_raw>>4) - ((int32_t)cal_data.dig_T1))) >> 12) *
				 ((int32_t)cal_data.dig_T3)) >> 14;
	temper_int = temp_t1 + temp_t2;
	bmp->temp = ((temper_int * 5 + 128) >> 8);
	bmp->temp /= 100.0f;
//-------------------Pressure----------------------------------------
	press_raw >>= 4;
	temp_p1 = ((int64_t) temper_int) - 128000;
	temp_p2 = temp_p1 * temp_p1 * (int64_t)cal_data.dig_P6;
	temp_p2 = temp_p2 + ((temp_p1 * (int64_t)cal_data.dig_P5) << 17);
	temp_p2 = temp_p2 + ((int64_t)cal_data.dig_P4 << 35);
	temp_p1 = ((temp_p1 * temp_p1 * (int64_t)cal_data.dig_P3) >> 8) + ((temp_p1 * (int64_t)cal_data.dig_P2) << 12);
	temp_p1 = (((((int64_t)1) << 47) + temp_p1)) * ((int64_t)cal_data.dig_P1) >> 33;
	if (temp_p1 == 0)
		bmp->press = 0; 									// avoid exception caused by division by zero
	p = 1048576 - press_raw;
	p = (((p << 31) - temp_p2) * 3125) / temp_p1;
	temp_p1 = (((int64_t)cal_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	temp_p2 = (((int64_t)cal_data.dig_P8) * p) >> 19;
	p = ((p + temp_p1 + temp_p2) >> 8) + ((int64_t)cal_data.dig_P7 << 4);
	pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
	bmp->press = pres_int / 100.0f;
//-------------------Humidity---------------------------------------
	hum_raw_sign = ((int32_t)hum_raw)&0x0000FFFF;
	
	v_x1_u32r = (temper_int - ((int32_t)76800));
	v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t)cal_data.dig_H4) << 20) -
	(((int32_t)cal_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	(((((((v_x1_u32r * ((int32_t)cal_data.dig_H6)) >> 10) *
	(((v_x1_u32r * ((int32_t)cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
	((int32_t)2097152)) * ((int32_t)cal_data.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((int32_t)cal_data.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	bmp->humidity = (v_x1_u32r>>12);
	bmp->humidity /= 1024.0f;	
}

float bme280_read_altitude(float sealevel)
{
  float att = 0.0f;
	float atm = bme280_read_pressure();
	att = 44330.0 * (1.0 - pow(atm / sealevel, 0.1903));	
	
  return att;
}



