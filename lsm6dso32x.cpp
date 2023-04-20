#include "config.h"

// try to skip the entire file if this is not defined, cuz Arduino always compiles everything...
#ifdef IMU_LSM6DSO32X

// Paul's NXPSensorFusion library has a nice general-purpose
// structure, so i've been adapting it to other sensors as I've
// switched chips a few times ...


#include "lsm6dso32x-pid/lsm6dso32x_reg.h"

#include "lsm6dso32x.h"
// #include <EEPROM.h>
// #include <util/crc16.h>
// #include <elapsedMillis.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

#include <SPI.h>
// defines SPI and SPI1; our IMU is on SPI1
// SPIPORT is defined in config.h ...

// NOTE: 
// "chip_addr" in SPI mode is the GPIO pin that asserts CS for the chip --
// kind of squeezing SPI's foot into I2C's shoe, but it fits.
	
const uint8_t chip_addr=SPI_cs;
// from datasheet:
const SPISettings spiconf(10000000, MSBFIRST, SPI_MODE3);


bool LSM6DSO32X_IMU::sleep() {
	uint8_t buf;
	//TODO: CTRL4_C bit 1 high enables gyroscope Sleep mode ?  is that better?
	
	_gyro_off();
	_accel_off();
	
	return true;
}

bool LSM6DSO32X_IMU::wake() {
	
	// _accel_on();
	// _gyro_on();
	//
	// seems to need the whole shebang to come back:
	return begin();
}

bool LSM6DSO32X_IMU::available() {
	update();
	return true;
}

void LSM6DSO32X_IMU::readMotionSensor(int& ax, int& ay, int& az, int& gx, int& gy, int& gz) {
	update();
	ax = accel_mag_raw[0];
	ay = accel_mag_raw[1];
	az = accel_mag_raw[2];
	gx = gyro_raw[0];
	gy = gyro_raw[1];
	gz = gyro_raw[2];
}

// not using ...
//
// #define G_PER_COUNT            0.0001220703125f  // = 1/8192
// #define UT_PER_COUNT           0.1f
// #define DEG_PER_SEC_PER_COUNT  0.0625f  // = 1/16
//
// void LSM6DSO32X_IMU::readMotionSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
// 	update();
// 	ax = (float)accel_mag_raw[0] * G_PER_COUNT;
// 	ay = (float)accel_mag_raw[1] * G_PER_COUNT;
// 	az = (float)accel_mag_raw[2] * G_PER_COUNT;
// 	gx = (float)accel_mag_raw[3] * DEG_PER_SEC_PER_COUNT;
// 	gy = (float)accel_mag_raw[4] * DEG_PER_SEC_PER_COUNT;
// 	gz = (float)accel_mag_raw[5] * DEG_PER_SEC_PER_COUNT;
// }
//
// #undef G_PER_COUNT
// #undef DEG_PER_SEC_PER_COUNT
// #undef UT_PER_COUNT

bool LSM6DSO32X_IMU::_accel_off(){
	//CTRL1_XL 0b0000xxxx : power down accelerometer 
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL1_XL, 0b00001000)) return false;
	return true;
}

bool LSM6DSO32X_IMU::_accel_on(){
	uint8_t buf;

  // // accel data rate & resolution: CTRL1_XL
  // if (!read_regs(chip_addr, LSM6DSO32X_CTRL1_XL, &buf, 1)) return false;
	// not getting valid result?
  Dbg_println(buf);

	// 10xx: 4g scale
	// xx0x: lpf disable
	// xxx0: ununsed
	buf = 0b00001000;

#ifdef IMU_6_666KHZ
  // rate = 6.66khz, res = +-8g
	buf = buf | 0b10100000;

# elif defined(IMU_3_333KHZ)
  // rate = 3.33khz, res = +-8g
	buf = buf | 0b10010000;
#else
  // rate = 1.66khz, res = +-8g
	buf = buf | 0b10000000;
#endif
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL1_XL, buf)) return false;
	return true;
}

bool LSM6DSO32X_IMU::_gyro_off(){
	uint8_t buf;

	//CTRL2_G 0b0000xxxx : power down gyroscope
  if (!read_regs(chip_addr, LSM6DSO32X_CTRL2_G, &buf, 1)) return false;
	buf = buf & 0b00001111 ; // mask the top four bits
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL2_G, buf)) return false;
	return true;
}

bool LSM6DSO32X_IMU::_gyro_on(){
	uint8_t buf;

  // gyro data rate & resolution:
#ifdef IMU_8KHZ
  // rate = 6.66khz, res = +-1000 deg/sec
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL2_G, 0b10101000)) return false;
#else
  // rate = 1.66khz, res = ?
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL2_G, 0b10001000)) return false;
#endif
	return true;

}

bool LSM6DSO32X_IMU::begin(){
	uint8_t buf;
  uint8_t temp; // actually signed ...
  uint8_t tempfrac;

  Dbg_print("LSM6DSO32X::begin: SPI slave on pin ");
  Dbg_println(chip_addr);

  // detect if chip is present
  if (!read_regs(chip_addr, LSM6DSO32X_WHO_AM_I, &buf, 1)) return false;
  Dbg_printf("LSM6DSO32X ID = %02X\n", buf);
  if ( (buf != 0x6CU) )
    return false;


  // // reset the device:
  // ctrl3_c: "software reset" (last bit)
	// defaults otherwise
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL3_C, 0b00000101)) return false;

  // wait for reset to complete
	// (datasheet says 35ms is "turn on time")
  delay(35);
  // detect if chip is still with us
  if (!read_regs(chip_addr, LSM6DSO32X_WHO_AM_I, &buf, 1)) return false;
  if ( (buf != 0x6CU) ) {
    Dbg_println("gone after reset!");
    return false;
	} else {
    Dbg_println("reset unit");
	}
	
	// configure interrupts to active low (defaults otherwise)
  if (!write_reg(chip_addr, LSM6DSO32X_CTRL3_C, 0b00100100)) return false;


  // get the temperature: OUT_TEMP_L/H
  // H is a signed integer celcius value, L is unsigned fractional 1/256ths of a degree celcius
  if (!read_regs(chip_addr, LSM6DSO32X_OUT_TEMP_H, &temp, 1)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUT_TEMP_L, &tempfrac, 1)) return false;
  Dbg_printf("temperature raw %i . %i\n", (int8_t)temp, tempfrac);
	// datasheet sec 4.3, table 4, teeny tiny footnote #3: temp sensor output is zero at 25 celcius ...
  Dbg_printf("temperature %f\n", ((int8_t)temp + 25 + ( tempfrac / 256.0)));

  // configure components:

  // disable FIFO
  if (!write_reg(chip_addr, LSM6DSO32X_FIFO_CTRL4, 0)) return false;

	if (! _accel_on()) return false;

	if (! _gyro_on()) return false;


  // configure interrupt 1 on accelerometer data ready
	if (!write_reg(chip_addr, LSM6DSO32X_INT1_CTRL, 0b00000001)) return false;


	//TODO: CTRL4_C bit 5 high: disable I2C (should i always do that? any savings?)
	
  Dbg_println("LSM6DSO32X configured");
  return true;
}

void LSM6DSO32X_IMU::update()
{
	// static elapsedMillis msec;
	int32_t alt;

	if (LSM6DSO32X_read(accel_mag_raw)) { // accel + mag
		//Dbg_println("accel+mag");
		// newdata = 1;
	}
}

// TOOO: if we don't use gyro data, we could add a method to just get accel.
bool LSM6DSO32X_IMU::LSM6DSO32X_read(int16_t *data)  // accel + mag
{
  uint8_t buf[13];

	// For all of these registers, the high byte lives right after the low byte, so we get them both by reading 2 bytes.
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_A, buf+1, 2)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_L_A, buf+3, 2)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_L_A, buf+5, 2)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_G, buf+7, 2)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTY_L_G, buf+9, 2)) return false;
  if (!read_regs(chip_addr, LSM6DSO32X_OUTZ_L_G, buf+11, 2)) return false;

	// for some reason, doing it this way is slower ...
  // if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_A, buf+1, 6)) return false;
  // if (!read_regs(chip_addr, LSM6DSO32X_OUTX_L_G, buf+7, 6)) return false;

  data[0] = (int16_t)((buf[2] << 8) | buf[1]);
  data[1] = (int16_t)((buf[4] << 8) | buf[3]);
  data[2] = (int16_t)((buf[6] << 8) | buf[5]);
  data[3] = (int16_t)((buf[8] << 8) | buf[7]);
  data[4] = (int16_t)((buf[10] << 8) | buf[9]);
  data[5] = (int16_t)((buf[12] << 8) | buf[11]);
	return true;
}

bool LSM6DSO32X_IMU::write_reg(uint8_t selector, uint8_t addr, uint8_t val)
{
	digitalWrite(selector, LOW);
	SPIPORT.transfer(addr & 0b01111111); // first bit clear for write-op
	SPIPORT.transfer(val);
	digitalWrite(selector, HIGH);
	return true; // what to check actually?
}

bool LSM6DSO32X_IMU::read_regs(uint8_t selector, uint8_t addr, uint8_t *data, uint8_t num)
{
	SPIPORT.beginTransaction(spiconf);
	digitalWrite(selector, LOW);
	SPIPORT.transfer(addr | 0b10000000); // first bit set for read-op
	while (num > 0) {
		*data++ = SPIPORT.transfer(0x00);
		num--;
	}
	digitalWrite(selector, HIGH);
	SPIPORT.endTransaction();
	return true;
}
// for debugging purposes mostly:
bool LSM6DSO32X_IMU::set_reg(uint8_t selector, uint8_t addr, uint8_t val)
{
	uint8_t buf;
	if (! read_regs(selector, addr, &buf, 1)) return false;
	if (buf == val) {
		Dbg_printf("addr %x was already %x\n", addr, val);
		return true;
	} else {
		Dbg_printf("changing addr %x from %x to %x\n", addr, buf, val);
		if (! write_reg(selector, addr, val)) return false;
		if (! read_regs(selector, addr, &buf, 1)) return false;
		if (buf != val) {
			Dbg_println("could not set register!");
			return false;
		}
		return true;
	}
}


// bool LSM6DSO32X_IMU::writeCalibration(const void *data)
// {
// 	const uint8_t *p = (const uint8_t *)data;
// 	uint16_t crc;
// 	uint8_t i;
//
// 	if (p[0] != 117 || p[1] != 84) return false;
// 	crc = 0xFFFF;
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		crc = _crc16_update(crc, p[i]);
// 	}
// 	if (crc != 0) return false;
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, p[i]);
// 	}
// 	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
// 		if (EEPROM.read(NXP_MOTION_CAL_EEADDR + i) != p[i]) return false;
// 	}
// 	memcpy(cal, ((const uint8_t *)data)+2, sizeof(cal));
// 	return true;
// }
//




#endif
