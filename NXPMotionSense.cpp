// hacked version for rev1, where the chip is actually an ICM-42605

#include "ICM42605/ICM42605_LIS2MDL_LPS22HB_Dragonfly/ICM42605.h"
#define ICM42605_I2C_ADDR0 0b1101000

#include "NXPMotionSense.h"
#include "utility/NXPSensorRegisters.h"
#include <util/crc16.h>
#include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

bool NXPMotionSense::begin()
{
	unsigned char buf[NXP_MOTION_CAL_SIZE];
	uint8_t i;
	uint16_t crc;

	Wire.begin();
	//Wire.setClock(400000);
	Wire.setClock(1000000); // ICM42605 supports 1mhz max i2c speed

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


static bool write_reg(uint8_t i2c, uint8_t addr, uint8_t val)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t i2c, uint8_t addr, uint8_t *data, uint8_t num)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	if (Wire.endTransmission(false) != 0) return false;
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

static bool read_regs(uint8_t i2c, uint8_t *data, uint8_t num)
{
	Wire.requestFrom(i2c, num);
	if (Wire.available() != num) return false;
	while (num > 0) {
		*data++ = Wire.read();
		num--;
	}
	return true;
}

bool NXPMotionSense::ICM42605_begin()
{
	const uint8_t i2c_addr=ICM42605_I2C_ADDR0;
	uint8_t b;
	uint8_t t[2];
	uint8_t reg;

	Serial.println("ICM42605_begin");

	// detect if chip is present
	if (!read_regs(i2c_addr, ICM42605_WHO_AM_I, &b, 1)) return false;
	Serial.printf("ICM42605 ID = %02X\n", b);
	if (b != 0x42) return false;

	// // reset the device:
	// if (!write_reg(i2c_addr, ICM42605_DEVICE_CONFIG, 0b00000001)) return false;    // 8khz
	// // wait 1ms for reset to complete
	// delay(2);
	// // detect if chip is still with us
	// if (!read_regs(i2c_addr, ICM42605_WHO_AM_I, &b, 1)) return false;
	// if (b != 0x42) {
	// 	Serial.println("gone after reset!");
	// 	return false;
	// } else {
	// 	Serial.println("reset unit");
	// }
	
	// interrupt electrical stuff: INT_CONFIG
	// int2 push pull
	reg = reg | 0b00010000;
	if (!write_reg(i2c_addr, ICM42605_INT_CONFIG, reg)) return false;  
	
	// report temperature: TEMP_DATA1/0
	// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
	if (!read_regs(i2c_addr, ICM42605_TEMP_DATA1, t, 2)) return false;
	Serial.printf("temperature raw %i / %i\n", t[0], t[1]);
	Serial.printf("temperature %f\n", ((t[0]<<8 + t[1]) / 132.48) + 25.0);

	// PWR_MGMT0: power up gyro and acc? manual says default is off ...
	// 0b00001111
	if (!write_reg(i2c_addr, ICM42605_PWR_MGMT0, 0b00001111)) return false;    
	
	// gyro: GYRO_CONFIG0
	// set output data rate to 8khz
	// GYRO_CONFIG0 = 0bxxxx0011
	// set sensitivity to +-2000dps
	// GYRO_CONFIG0 = 0b000.....
	// if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000011)) return false;    // 8khz
	//if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000100)) return false;    // 4khz
	if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000101)) return false;    // 2khz
	
	// accelerometer: ACCEL_CONFIG0
	// set output data rate to 8khz
	// ICM42605_ACCEL_CONFIG0 = 0b....0011
	// set accel sensitivity to 8g
	// ICM42605_ACCEL_CONFIG0 = 0b001.....
	// to do both in one register:
	//if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100011)) return false;    // 8khz
	//if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100100)) return false;    // 4khz
	if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100101)) return false;  // 2khz
	//if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100110)) return false;  // 1khz

	// Interrupt stuff:
	//
	// INT_CONFIG1:
	// interrupt pulse duration stuff for higher data rates
	// "6: 1= Interrupt pulse duration is 8 μs. Required if ODR ≥ 4kHz, optional for ODR < 4kHz."
	// "5: 1: Disables de-assert duration. Required if ODR ≥ 4kHz, optional for ODR < 4kHz.
	// "4: User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
	// INT_CONFIG1 = 0b01100000
	if (!write_reg(i2c_addr, ICM42605_INT_CONFIG1, 0b01100000)) return false;     // 8us
	// if (!write_reg(i2c_addr, ICM42605_INT_CONFIG1, 0b00000000)) return false;     // 100us
	// if (!read_regs(i2c_addr, ICM42605_INT_CONFIG1, &reg, 1)) return false;
	// Serial.printf("INT_CONFIG1: %x\n", reg);
	
	// interrupt INT2 on data ready:
	// INT_SOURCE3 = 0b00001000
	if (!write_reg(i2c_addr, ICM42605_INT_SOURCE3, 0b00001000)) return false;    // int2 on data ready
	if (!read_regs(i2c_addr, ICM42605_INT_SOURCE3, &reg, 1)) return false;
	Serial.printf("INT_SOURCE3: %x\n", reg);

	// // make sure we're not self-testing ...
	// if (!read_regs(i2c_addr, ICM42605_SELF_TEST_CONFIG, &reg, 1)) return false;
	// Serial.printf("SELF_TEST_CONFIG: %x\n", reg);
	// we're not.
	
	// // signal path reset?
	// if (!read_regs(i2c_addr, ICM42605_SIGNAL_PATH_RESET, &reg, 1)) return false;
	// Serial.printf("SIGNAL_PATH_RESET: %x\n", reg);
	// if (!write_reg(i2c_addr, ICM42605_SIGNAL_PATH_RESET, 0b00001000)) return false;   
	// not necessary atm.


	Serial.println("ICM42605 Configured");
	return true;
}

bool NXPMotionSense::ICM42605_read(int16_t *data)  // accel + mag
{
	// static elapsedMicros usec_since;
	// static int32_t usec_history=5000;
	const uint8_t i2c_addr=ICM42605_I2C_ADDR0;
	uint8_t buf[13];
  //
	// int32_t usec = usec_since;


	// LATEST THEORY is that we need to first read the 
	// data registers, then read the INT_STATUS in order
	// to reset the interrupt flag and arm the next interrupt.
	// Otherwise we might get a second interrupt during the read;
	// that might explain glitches .
	// TODO: try swapping those two read_regs.
	
	
	// This seems unnnecessary now that we're interrupt driven,
	// but when I take it out all sorts of weird shit happens:
	// we get like 100 more "hits" (interrupts), and we also 
	// start to see accelerometer glitches.  Maybe the interrupt
	// is somehow double-firing?  Consider learning more about
	// the interrupt config registers in the IMU, and the arduino
	// pin-listening options in attachIntrrupt ...
	// 
	//if (usec + 100 < usec_history) return false;
	//if (usec + 10 < usec_history) return false;
	//if (usec + 1 < usec_history) return false;

	// This also shouldn't be needed when interrupt driven:
	//
	// I think we're looking for "data ready" here ... that's the 
	// data_rdy_int bit from the INT_STATUS reg
// 	if (!read_regs(i2c_addr, ICM42605_INT_STATUS, buf, 1)) return false;
// #define DATA_RDY_INT 0b00001000
// 	if (!(buf[0] & DATA_RDY_INT )) return false;

	// usec_since -= usec;
	// int diff = (usec - usec_history) >> 3;
	// if (diff < -15) diff = -15;
	// else if (diff > 15) diff = 15;
	// usec_history += diff;

	// the registers for 6 bytes of acc and 6 bytes of gyro are all contiguous here:
	if (!read_regs(i2c_addr, ICM42605_ACCEL_DATA_X1 , buf+1, 12)) return false;

	//if (!read_regs(i2c_addr, buf, 13)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	// // clear flags
	// if (!read_regs(i2c_addr, ICM42605_INT_STATUS, buf, 1)) return false;
	// return true;
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
