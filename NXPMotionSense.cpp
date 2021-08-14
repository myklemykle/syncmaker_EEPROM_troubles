// hacked for ICM-42605

#include "config.h"

#include "ICM42605/ICM42605_LIS2MDL_LPS22HB_Dragonfly/ICM42605.h"
#define ICM42605_I2C_ADDR0 0b1101000
#define ICM42605_SPI_CS 10;

#include "NXPMotionSense.h"
#include <EEPROM.h>
#include "utility/NXPSensorRegisters.h"
#include <util/crc16.h>
#include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

#ifdef IMU_SPI
#include <SPI.h>
	// default spiconf should be fine for this chip
const uint8_t chip_addr=ICM42605_SPI_CS;
const SPISettings spiconf(24000000, MSBFIRST, SPI_MODE0);
//const SPISettings spiconf(4000000, MSBFIRST, SPI_MODE0);
#else
#include <Wire.h>
const uint8_t chip_addr=ICM42605_I2C_ADDR0;
#endif 

bool NXPMotionSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;

#ifdef IMU_SPI
	SPI.begin();
#else
	Wire.begin();
	Wire.setClock(1000000); // ICM42605 supports 1mhz max i2c speed
#endif

	memset(accel_mag_raw, 0, sizeof(accel_mag_raw));
	memset(gyro_raw, 0, sizeof(gyro_raw));

	//Serial.println("init hardware");
	while (!ICM42605_begin()) {
		Serial.println("config error ICM42605");
		delay(1000);
	}

	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 50.0f;
	}
	return true;

}


void NXPMotionSense::update()
{
	static elapsedMillis msec;
	int32_t alt;

	if (ICM42605_read(accel_mag_raw)) { // accel + mag
		//Serial.println("accel+mag");
		newdata = 1;
	}
}


static bool write_reg(uint8_t selector, uint8_t addr, uint8_t val)
{
#ifdef IMU_SPI
	digitalWrite(selector, LOW);
	SPI.transfer(addr & 0b01111111); // first bit clear for write-op
	SPI.transfer(val);
	digitalWrite(selector, HIGH);
#else
	Wire.beginTransmission(selector);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
#endif
}

static bool read_regs(uint8_t selector, uint8_t addr, uint8_t *data, uint8_t num)
{
#ifdef IMU_SPI
	SPI.beginTransaction(spiconf);
	digitalWrite(selector, LOW);
	SPI.transfer(addr | 0b10000000); // first bit set for read-op
	while (num > 0) {
		*data++ = SPI.transfer(0x00);
		num--;
	}
	digitalWrite(selector, HIGH);
	SPI.endTransaction();
#else
	Wire.beginTransmission(selector);
	Wire.write(addr);
	if (Wire.endTransmission(false) != 0) return false;
	Wire.requestFrom(selector, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
#endif
	return true;
}


bool NXPMotionSense::ICM42605_sleep(){
	// 0b00100000
	if (!write_reg(chip_addr, ICM42605_PWR_MGMT0, 0b00100000)) return false;    
	return true;
}
bool NXPMotionSense::ICM42605_wake(){
	// PWR_MGMT0: power up gyro and acc. 
	if (!write_reg(chip_addr, ICM42605_PWR_MGMT0, 0b00001111)) return false;    
	return true;
}
bool NXPMotionSense::ICM42605_begin()
{
	uint8_t b;
	uint8_t t[2];
	uint8_t reg;

#ifdef IMU_SPI
	Serial.print("ICM42605_begin: SPI bus on pin ");
	Serial.println(chip_addr);
#else
	Serial.print("ICM42605_begin: I2c bus addr ");
	Serial.println(chip_addr);
#endif

	// detect if chip is present
	if (!read_regs(chip_addr, ICM42605_WHO_AM_I, &b, 1)) return false;
	Serial.printf("ICM42605 ID = %02X\n", b);
	if (b != 0x42) return false;

	// // reset the device:
	// if (!write_reg(chip_addr, ICM42605_DEVICE_CONFIG, 0b00000001)) return false;    // 8khz
	// // wait 1ms for reset to complete
	// delay(2);
	// // detect if chip is still with us
	// if (!read_regs(chip_addr, ICM42605_WHO_AM_I, &b, 1)) return false;
	// if (b != 0x42) {
	// 	Serial.println("gone after reset!");
	// 	return false;
	// } else {
	// 	Serial.println("reset unit");
	// }
	
	// interrupt electrical stuff: INT_CONFIG
	// int1 & int2 push pull
	reg = reg | 0b00010010;
	if (!write_reg(chip_addr, ICM42605_INT_CONFIG, reg)) return false;  
	
	// report temperature: TEMP_DATA1/0
	// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
	if (!read_regs(chip_addr, ICM42605_TEMP_DATA1, t, 2)) return false;
	Serial.printf("temperature raw %i / %i\n", t[0], t[1]);
	Serial.printf("temperature %f\n", ((t[0]<<8 + t[1]) / 132.48) + 25.0);

	// PWR_MGMT0: power up gyro and acc. Leave temperature sensor off.
	// 0b00001111
	if (!write_reg(chip_addr, ICM42605_PWR_MGMT0, 0b00001111)) return false;    
	
	// gyro: GYRO_CONFIG0
	// set output data rate ...
	// GYRO_CONFIG0 = 0bxxxx0011
	// set sensitivity to +-2000dps
	// GYRO_CONFIG0 = 0b000.....
#ifdef IMU_8KHZ
	if (!write_reg(chip_addr, ICM42605_GYRO_CONFIG0, 0b00000011)) return false;    // 8khz
#else
	// if (!write_reg(chip_addr, ICM42605_GYRO_CONFIG0, 0b00000100)) return false;    // 4khz
	if (!write_reg(chip_addr, ICM42605_GYRO_CONFIG0, 0b00000101)) return false;    // 2khz
#endif
	
	// accelerometer: ACCEL_CONFIG0
	// set output data rate ...
	// ICM42605_ACCEL_CONFIG0 = 0b....0011
	// set accel sensitivity to 8g
	// ICM42605_ACCEL_CONFIG0 = 0b001.....
	// to do both in one register:
#ifdef IMU_8KHZ
	if (!write_reg(chip_addr, ICM42605_ACCEL_CONFIG0, 0b00100011)) return false;    // 8khz
#else
	//if (!write_reg(chip_addr, ICM42605_ACCEL_CONFIG0, 0b00100100)) return false;    // 4khz
	if (!write_reg(chip_addr, ICM42605_ACCEL_CONFIG0, 0b00100101)) return false;  // 2khz
#endif

#ifdef IMU_INTERRUPTS
	// Interrupt stuff:
	//
	// INT_CONFIG1:
	// interrupt pulse duration stuff for higher data rates
	// "6: 1= Interrupt pulse duration is 8 μs. Required if ODR ≥ 4kHz, optional for ODR < 4kHz."
	// "5: 1: Disables de-assert duration. Required if ODR ≥ 4kHz, optional for ODR < 4kHz.
	// "4: User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
	// INT_CONFIG1 = 0b01100000
	if (!write_reg(chip_addr, ICM42605_INT_CONFIG1, 0b01100000)) return false;     // 8us
	// if (!write_reg(chip_addr, ICM42605_INT_CONFIG1, 0b00000000)) return false;     // 100us
	// if (!read_regs(chip_addr, ICM42605_INT_CONFIG1, &reg, 1)) return false;
	// Serial.printf("INT_CONFIG1: %x\n", reg);
	
	// interrupt INT1 and INT2 on data ready:
	// INT_SOURCE0 = 0b00001000
	if (!write_reg(chip_addr, ICM42605_INT_SOURCE0, 0b00001000)) return false;    // int2 on data ready
	if (!read_regs(chip_addr, ICM42605_INT_SOURCE0, &reg, 1)) return false;
	Serial.printf("INT_SOURCE0: %x\n", reg);
	// INT_SOURCE3 = 0b00001000
	if (!write_reg(chip_addr, ICM42605_INT_SOURCE3, 0b00001000)) return false;    // int2 on data ready
	if (!read_regs(chip_addr, ICM42605_INT_SOURCE3, &reg, 1)) return false;
	Serial.printf("INT_SOURCE3: %x\n", reg);
#endif

	// // make sure we're not self-testing ...
	// if (!read_regs(chip_addr, ICM42605_SELF_TEST_CONFIG, &reg, 1)) return false;
	// Serial.printf("SELF_TEST_CONFIG: %x\n", reg);
	// we're not.
	
	// // signal path reset?
	// if (!read_regs(chip_addr, ICM42605_SIGNAL_PATH_RESET, &reg, 1)) return false;
	// Serial.printf("SIGNAL_PATH_RESET: %x\n", reg);
	// if (!write_reg(chip_addr, ICM42605_SIGNAL_PATH_RESET, 0b00001000)) return false;   
	// not necessary atm.


	Serial.println("ICM42605 Configured");
	return true;
}

bool NXPMotionSense::ICM42605_read(int16_t *data)  // accel + mag
{
	uint8_t buf[13];

	// the registers for 6 bytes of acc and 6 bytes of gyro are all contiguous here:
	if (!read_regs(chip_addr, ICM42605_ACCEL_DATA_X1 , buf+1, 12)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
}


bool NXPMotionSense::writeCalibration(const void *data)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t crc;
	uint8_t i;

	if (p[0] != 117 || p[1] != 84) return false;
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, p[i]);
	}
	if (crc != 0) return false;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
	}
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
	}
	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
	return true;
}
