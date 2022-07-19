/**
 * @author Christoph Kolhoff
 * @file lsm9ds1.cpp
 */

#include <iostream>

#include "interfacesHardware.h"
#include "lsm9ds1.h"


/* ================================================================================
 * SPI Interface
 * ================================================================================ */

/**
 * @brief Constructor
 */
Lsm9ds1Spi::Lsm9ds1Spi(void)
{
#ifdef __unix

#endif // __unix
}

/**
 * @brief Destructor
 */
Lsm9ds1Spi::~Lsm9ds1Spi(void)
{
#ifdef __unix

#endif // __unix
}

/**
 * @brief Initializes the SPI interface to be used with the LSM9DS1 IMU. The used SPI mode is CPOL = 0 and CPHA = 0.
 * @param[in] filename File specifying the SPI-device for the interface
 * @param[in] speed Clock speed [Hz]
 * @return -1 on failure; non-negative on success
 */
int8_t
Lsm9ds1Spi::init(char *filename, int speed)
{
    int8_t ret = -1;
#ifdef __unix
    ret = _interface->openDevice(filename, 0, speed, 8, 0, 0);
#endif // __unix

    resetDevice();

    return ret;
}

/**
 * @brief Reset all present configurations of the device
 * @post Device is in Configuration Mode
 */
void
Lsm9ds1Spi::resetDevice(void)
{
    int8_t ret = -1;

    uint8_t bytesIn[2] = {0x22, 1};
    uint8_t bytesOut[2] = {0, 0};

#ifdef __unix
    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
}

/**
 * @brief Shutdown the device and disconect device
 * @return -1 on failure; non-negative on success
 * @attention This method has to be called when the interface is not longer needed
 */
int8_t
Lsm9ds1Spi::terminate(void)
{
    int8_t ret = -1;
#ifdef __unix
    ret = _interface->closeDevice();
#endif // __unix
    return ret;
}

/**
 * @brief Read the value stored in the WHO_AM_I register. This is for testing purpose only.
 * @return Value stored in the register
 */
uint8_t
Lsm9ds1Spi::getWhoAmI(void)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {0, 0};
    uint8_t bytesOut[2] = {0, 0};

    bytesIn[0] = 0x0F | 0x80;

    _interface->readWriteBytes(bytesOut, bytesIn, 2);

    ret = *(bytesOut + 1);
#endif // __unix
    return ret;
}

/**
 * @brief Set the value of the register CTRL_REG1_G, see manual p. 45.
 * @param value Value to be set
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::set_CTRL_REG1_G(uint8_t value)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {};
    bytesIn[0] = 0x10;
    bytesIn[1] =value;

    uint8_t bytesOut[2] = {0, 0};

    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
    return ret;
}

/**
 * @brief Set the value of the register CTRL_REG6_XL, see manual pp. 51f.
 * @param value Value to be set
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::set_CTRL_REG6_XL(uint8_t value)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {};
    bytesIn[0] = 0x20;
    bytesIn[1] =value;

    uint8_t bytesOut[2] = {0, 0};

    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
    return ret;
}

/**
 * @brief Set the value of the register CTRL_REG8, see manual p. 53.
 * @param value Value to be set
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::set_CTRL_REG8(uint8_t value)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {};
    bytesIn[0] = 0x22;
    bytesIn[1] =value;

    uint8_t bytesOut[2] = {0, 0};

    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
    return ret;
}

/**
 * @brief Set the value of the register CTRL_REG9, see manual p. 54.
 * @param value Value to be set
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::set_CTRL_REG9(uint8_t value)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {};
    bytesIn[0] = 0x23;
    bytesIn[1] =value;

    uint8_t bytesOut[2] = {0, 0};

    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
    return ret;
}

/**
 * @brief Set the value of the register FIFO_CTRL, see manual p. 56.
 * @param value Value to be set
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::set_FIFO_CTRL(uint8_t value)
{
    uint8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[2] = {0, 0};
    bytesIn[0] = 0x2E;
    bytesIn[1] =value;

    uint8_t bytesOut[2] = {0, 0};

    ret = _interface->readWriteBytes(bytesOut, bytesIn, 2);
#endif // __unix
    return ret;
}

/**
 * @brief Read the current value of the acceleration along the x-axis
 * @return Value of the acceleration [m/s^2]
 */
float
Lsm9ds1Spi::getAccX(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x28, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on x-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.000061; // compute value

    return value;
}

/**
 * @brief Read the current value of the acceleration along the x-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getAccXCompl(uint8_t* data)
{
    int8_t ret = readReg(0x28, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on x-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Read the current value of the acceleration along the y-axis
 * @return Value of the acceleration [m/s^2]
 */
float
Lsm9ds1Spi::getAccY(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x2A, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on y-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.000061; // compute value

    return value;
}

/**
 * @brief Read the current value of the acceleration along the y-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getAccYCompl(uint8_t* data)
{
    int8_t ret = readReg(0x2A, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on y-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Read the current value of the acceleration along the z-axis
 * @return Value of the acceleration [m/s^2]
 */
float
Lsm9ds1Spi::getAccZ(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x2C, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on z-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.000061; // compute value

    return value;
}

/**
 * @brief Read the current value of the acceleration along the z-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getAccZCompl(uint8_t* data)
{
    int8_t ret = readReg(0x2C, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading acceleration on z-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Read the current value of the angular velocity along the x-axis
 * @return  Value of the angular velocity [°/s]
 */
float
Lsm9ds1Spi::getGyrX(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x18, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around x-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.00875; // compute value

    return value;
}

/**
 * @brief Read the current value of the angular velocity along the x-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getGyrXCompl(uint8_t* data)
{
    int8_t ret = readReg(0x18, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around x-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Read the current value of the angular velocity along the y-axis
 * @return  Value of the angular velocity [°/s]
 */
float
Lsm9ds1Spi::getGyrY(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x1A, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around y-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.00875; // compute value

    return value;
}

/**
 * @brief Read the current value of the angular velocity along the y-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getGyrYCompl(uint8_t* data)
{
    int8_t ret = readReg(0x1A, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around y-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Read the current value of the angular velocity along the z-axis
 * @return  Value of the angular velocity [°/s]
 */
float
Lsm9ds1Spi::getGyrZ(void)
{
    // value to return
    float value = 0.0;
    int fusedValue = 0;

    // reading highand low byte
    uint8_t complBytes[2] = {};
    int8_t ret = readReg(0x1C, complBytes, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around z-axis" << endl;
#endif // __unix
    }

    fusedValue = (*complBytes << 8) | (*(complBytes + 1));
    value = fusedValue * 0.00875; // compute value

    return value;
}

/**
 * @brief Read the current value of the angular velocity along the z-axis as a complement of twos
 * @param[out] data Array to pass data to
 */
void
Lsm9ds1Spi::getGyrZCompl(uint8_t* data)
{
    int8_t ret = readReg(0x1C, data, 2);
    if(0 > ret) {
#ifdef __unix
        cout << "Error reading angular velocity around z-axis" << endl;
#endif // __unix
    }
}

/**
 * @brief Reads the value of a given register
 * @param[in] reg Address of the register to start reading from
 * @param[out] dataReg Array to store read data in
 * @param[in] nBytes Number of bytes to read (1-254)
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::readReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes)
{
    int8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[255] = {};
    uint8_t bytesOut[255] = {};

    bytesIn[0] = reg | 0x80;

    ret = _interface->readWriteBytes(bytesOut, bytesIn, nBytes + 1);
    if(0 > ret) {
        printf("Failed to read LSM9DS1 register %u\n\r", reg);
        terminate();
    }

    // parse read values to array
    for(uint8_t indByte = 0; indByte < nBytes; indByte++) {
        *(dataReg + indByte) = *(bytesOut + indByte + 1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Writes the value to a given register
 * @param[in] reg Address of the first register to write to
 * @param[in] val Value to be set
 * @param[in] nBytes Length of val (1-254)
 * @return -1 on failure, 0 on success
 */
int8_t
Lsm9ds1Spi::writeReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes)
{
    int8_t ret = -1;

#ifdef __unix
    uint8_t bytesIn[255] = {};
    uint8_t bytesOut[255] = {};

    // parse the register address to the buffer
    bytesIn[0] = reg;

    // parse values to array to be send
    for(uint8_t indByte = 0; indByte < nBytes; indByte++) {
        *(bytesIn + indByte + 1) = *(dataReg + indByte);
    }

    ret = _interface->readWriteBytes(bytesOut, bytesIn, nBytes + 1);
#endif // __unix

    return ret;
}

/* ================================================================================
 * I2C Interface
 * ================================================================================ */

