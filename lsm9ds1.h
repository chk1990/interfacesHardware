/**
 * @author Christoph Kolhoff
 * @file lsm9ds1.h
 */

#ifndef _LSM9DS1_H_
#define _LSM9DS1_H_

#include "interfacesHardware.h"
#include "group_names_doxy.h"

/**
  * @addtogroup devices
  * @{
  */

/**
 * @class Lsm9ds1Spi
 * @brief This class provides an interface to use the ST Microelectronics IMU LSM9DS1. The magnitude's range is 2g.
 * @author Christoph Kolhoff
 * @par User API example for receive and transmit
 * @code
 * #include <iostream>
 * #include "lsm9ds1.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *
 *      // Configure the SPI interface
 *      char *interfaceFile = (char*)"/dev/spidev0.1";
 *      int speed = 1000000; // SCKL speed
 *
 *      // create and configure IMU object
 *      Lsm9ds1Spi Imu;
 *      Imu.init(interfaceFile, speed);
 *
 *      // write one byte to a register
 *      uint8_t ret = -1;
 *      uint8_t data[2] = {0b10101010, 0};
 *      ret = Imu.writeReg(0x10, data, 1);
 *
 *      // read one byte from a register
 *      ret = -1;
 *      ret = Imu.readReg(0x10, data, 1);
 *
 *      // close and disconnect
 *      Imu.terminate();
 *
 *      printf("Returned value: %u\n\r", ret);
 *      printf("Returned data: %u\n\r", data[0]);
 *
 *      return 0;
 * }
 * @endcode
 * @todo Adapt for other ranges of measurement
 */
class Lsm9ds1Spi {
public:
    // constructors and destructors
    Lsm9ds1Spi(void);
    ~Lsm9ds1Spi(void);

    // initialization and termination
    int8_t init(char *filename, int speed);
    void resetDevice(void);
    int8_t terminate(void);

    // configuration
    int8_t set_CTRL_REG1_G(uint8_t value);
    int8_t set_CTRL_REG6_XL(uint8_t value);
    int8_t set_CTRL_REG8(uint8_t value);
    int8_t set_CTRL_REG9(uint8_t value);
    int8_t set_FIFO_CTRL(uint8_t value);

    // read and write registers
    int8_t readReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes);
    int8_t writeReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes);

    // read specific registers
    uint8_t getWhoAmI(void);

    float getAccX(void);
    void getAccXCompl(uint8_t* data);
    float getAccY(void);
    void getAccYCompl(uint8_t* data);
    float getAccZ(void);
    void getAccZCompl(uint8_t* data);
    float getGyrX(void);
    void getGyrXCompl(uint8_t* data);
    float getGyrY(void);
    void getGyrYCompl(uint8_t* data);
    float getGyrZ(void);
    void getGyrZCompl(uint8_t* data);

private:
    // member types
#ifdef __unix
    ISpi_Master* _interface = new ISpi_Master; ///< SPI interface to be used
#endif // __unix

};

/**
 * @class Lsm9ds1I2c
 * @brief This class provides an interface to use the ST Microelectronics IMU LSM9DS1
 * @author Christoph Kolhoff
 * @par User API example for receive and transmit
 * @code
 * #include <iostream>
 * #include "lsm9ds1.h"
 *
 * using namespace std;
 *
 * int
 * main(void) {
 *      return 0;
 * }
 * @endcode
 * @todo complete if needed
 */
class Lsm9ds1I2c {
public:
    Lsm9ds1I2c(void);
    ~Lsm9ds1I2c(void);
private:
    // member types
#ifdef __unix
    II2c_Master* _interface = new II2c_Master; ///< I2C interface to be used
#endif // __unix
};

/**
  * @}
  */ // end of group devices

#endif // _LSM9DS1_H_
