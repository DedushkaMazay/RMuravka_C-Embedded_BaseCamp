#include "i2c1control.h"
#include "main.h"
#include "math.h"

I2C_HandleTypeDef hi2c1;
uint8_t devId = 0x80, firstEN = 0;
uint8_t TxBuff[2] = { 0 };
uint16_t DCycle = 199, address = 0x06;

void turnON(void) {

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	setNumberLed(address/4);

}

void turnOFF(void) {

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

}

void setFreq(uint16_t PWMFreq) {
	uint16_t Prescale = (int) round((25000000.f / (4096 * PWMFreq)) - 1);

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	TxBuff[0] = 0xFE;
	TxBuff[1] = Prescale;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	TxBuff[0] = 0x00;
	TxBuff[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	TxBuff[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	if (firstEN)
		setNumberLed(address/4);
}

void setDuty(uint16_t DutyCycle) {
	DCycle = (int) round(((4096.f / 100) * DutyCycle) - 1);
	if (firstEN)
		setNumberLed(address/4);
}

void setNumberLed(uint16_t led) {
	if (firstEN){
		TxBuff[0] = address + 3;
		TxBuff[1] = 0x10;
		HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
	}
	firstEN = 1;
	if (led != 0xfa)
		address = 0x06 + (led - 1) * 4;
	else
		address = 0xfa;

	TxBuff[0] = address;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	TxBuff[0]++;
	TxBuff[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	TxBuff[0]++;
	TxBuff[1] = DCycle % 256;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);

	TxBuff[0]++;
	TxBuff[1] = DCycle / 256;
	HAL_I2C_Master_Transmit(&hi2c1, devId, TxBuff, 2, 1000);
}

