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
	// Wire.setClock(400000);
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

	Serial.println("ICM42605_begin");

	// detect if chip is present
	if (!read_regs(i2c_addr, ICM42605_WHO_AM_I, &b, 1)) return false;
	Serial.printf("ICM42605 ID = %02X\n", b);
	if (b != 0x42) return false;

	// accelerometer:
	// set output data rate to 8khz
	// ICM42605_ACCEL_CONFIG0 = 0b001.....
	// set accel sensitivity to 8g
	// ICM42605_ACCEL_CONFIG0 = 0b....0011
	// to do both in one register:
	if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100011)) return false;    // 8khz
	// if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100100)) return false;    // 4khz
	// if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100101)) return false;  // 2khz
	//if (!write_reg(i2c_addr, ICM42605_ACCEL_CONFIG0, 0b00100110)) return false;  // 1khz
	
	// gyro: 
	// set output data rate to 8khz
	// GYRO_CONFIG0 = 0bxxxx0011
	// set sensitivity to +-2000dps
	// GYRO_CONFIG0 = 0b000.....
	if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000011)) return false;    // 8khz
	// if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000100)) return false;    // 4khz
	// if (!write_reg(i2c_addr, ICM42605_GYRO_CONFIG0, 0b00000101)) return false;    // 2khz

	// interrupt INT2 on data ready:
	// INT_SOURCE3 = 0b00001000
	if (!write_reg(i2c_addr, ICM42605_INT_SOURCE3, 0b00001000)) return false;    // int2 on data ready

	Serial.println("ICM42605 Configured");
	return true;
}

bool NXPMotionSense::ICM42605_read(int16_t *data)  // accel + mag
{
	static elapsedMicros usec_since;
	static int32_t usec_history=5000;
	const uint8_t i2c_addr=ICM42605_I2C_ADDR0;
	uint8_t buf[13];

	int32_t usec = usec_since;

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
	if (usec + 1 < usec_history) return false;

	// This also shouldn't be needed when interrupt driven:
	//
	// I think we're looking for "data ready" here ... that's the 
	// data_rdy_int bit from the INT_STATUS reg
	if (!read_regs(i2c_addr, ICM42605_INT_STATUS, buf, 1)) return false;
#define DATA_RDY_INT 0b00001000
	if (!(buf[0] & DATA_RDY_INT )) return false;
	// if (!buf[0]) return false;
	// if (buf[0]!=0)
	// 	Serial.println(buf[0]);

	usec_since -= usec;
	int diff = (usec - usec_history) >> 3;
	if (diff < -15) diff = -15;
	else if (diff > 15) diff = 15;
	usec_history += diff;

	// the registers for 6 bytes of acc and 6 bytes of gyro are all contiguous here:
	if (!read_regs(i2c_addr, ICM42605_ACCEL_DATA_X1 , buf+1, 12)) return false;

	//if (!read_regs(i2c_addr, buf, 13)) return false;

	data[0] = (int16_t)((buf[1] << 8) | buf[2]);
	data[1] = (int16_t)((buf[3] << 8) | buf[4]);
	data[2] = (int16_t)((buf[5] << 8) | buf[6]);
	data[3] = (int16_t)((buf[7] << 8) | buf[8]);
	data[4] = (int16_t)((buf[9] << 8) | buf[10]);
	data[5] = (int16_t)((buf[11] << 8) | buf[12]);
	return true;
}

bool NXPMotionSense::FXOS8700_begin()
{
	const uint8_t i2c_addr=FXOS8700_I2C_ADDR0;
	uint8_t b;

	Serial.println("FXOS8700_begin");

	// detect if chip is present
	if (!read_regs(i2c_addr, FXOS8700_WHO_AM_I, &b, 1)) return false;
	Serial.printf("FXOS8700 ID = %02X\n", b);
	if (b != 0xC7) return false;

	// place into standby mode
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0)) return false;

	// configure magnetometer
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG1, 0x1F)) return false;  // M + A (hybrid) mode
	//if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG1, 0x1C)) return false;  // A only
	if (!write_reg(i2c_addr, FXOS8700_M_CTRL_REG2, 0x20)) return false;

	// configure accelerometer
	if (!write_reg(i2c_addr, FXOS8700_XYZ_DATA_CFG, 0x01)) return false; // 4G range, hpf off, 
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG2, 0x02)) return false; // hires power mode, no oversampling

	// configure frequency & enable
	//if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0x15)) return false; // 100Hz A+M, hipass, active mode.
	//if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0x0D)) return false; // 200Hz A+M, hipass, active mode.
	if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0b00001101)) return false; // 200Hz A+M, hipass, active mode.
	//if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0b00001001)) return false; // 200Hz A+M, wideband, active mode.
	//if (!write_reg(i2c_addr, FXOS8700_CTRL_REG1, 0b00000001)) return false; // 400Hz A+M, wideband, active mode.
	
	Serial.println("FXOS8700 Configured");
	return true;
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
