/**
 * @author Christoph Kolhoff
 * @file interfacesHardware.cpp
 */

#include "interfacesHardware.h"

#ifdef __unix
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <netdb.h>
#include <net/if.h>
#include <netinet/in.h> // for internet domain addresses
#include <strings.h> // for bzero - https://man7.org/linux/man-pages/man3/bzero.3.html
#include <sys/ioctl.h> // accessing some unix/linux kernel functions
#include <sys/select.h>
#include <sys/socket.h> // structures and other types for sockets - https://pubs.opengroup.org/onlinepubs/007908775/xns/syssocket.h.html
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>
#endif // __unix

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;

// =========================================================
// IGpio
// =========================================================

// constructors and destructors ----------------------------

/**
 * @brief Setting up the internal device of the OS using "/sys/class/gpio/export", "/sys/class/gpio/unexport" and "/sys/class/gpio/"
 */
IGpio::IGpio()
{
#ifdef __unix
    _filenamePortOpen = "/sys/class/gpio/export";
    _filenamePortClose = "/sys/class/gpio/unexport";
    _pathGpio = "/sys/class/gpio/";
#endif // __unix

    init();
}

#ifdef __unix
/**
 * @brief Setting up the internal device of the OS
 * @param[in] filenameOpen File specifying the device for the interface
 * @param[in] filenameClose File specifying the device for the interface
 * @param[in] pathGpio Path where GPIO files of the OS are located
 */
IGpio::IGpio(char *filenameOpen, char *filenameClose, char *pathGpio)
{
    _filenamePortOpen = filenameOpen;
    _filenamePortClose = filenameClose;
    _pathGpio = pathGpio;
}
#endif // __unix

/**
 * @brief Destructor
 * @todo Destroy all exported files when aborted
 */
IGpio::~IGpio()
{
    //deinit();
}

// configuration methods -----------------------------------

/**
 * @brief Open GPIO port
 * @return -1 on failure; non-negative on success
 */
int
IGpio::init(void)
{
#ifdef __unix
    // change user permissions
    string temp = "sudo chmod 777 " + std::string(_filenamePortOpen);
    const char *commandOpen = temp.c_str();
    system(commandOpen);

    temp = "sudo chmod 777 " + std::string(_filenamePortClose);
    const char *commandClose = temp.c_str();
    system(commandClose);


    _fdOpen = open(_filenamePortOpen, O_RDWR); // O_WRONLY
    if(0 > _fdOpen) {
        deinit();
        perror("Error opening GPIO port");
        exit(1);
    }

    return _fdOpen;
#endif // __unix
}

/**
 * @brief Close GPIO port
 * @return -1 on failure; non-negative on success
 * @todo close all ports
 * @todo check if written number of signs in loop is valid
 */
int
IGpio::deinit(void)
{
#ifdef __unix
    // Open file to kill GPIO files
    _fdClose = open(_filenamePortClose, O_RDWR);
    if(0 > _fdClose) {
        close(_fdClose);
        perror("Error closing GPIO port");
        exit(1);
    }

    string portMsg;
    string pinTemp;

    for(uint8_t indGpio = 0; indGpio < _gpioBcm.size(); indGpio++) {

        const char *pinStr;

        pinTemp = to_string(_gpioBcm.at(indGpio));
        portMsg = "Error closing GPIO port " + pinTemp;

        pinStr = pinTemp.c_str();
        _nChars = (int)sizeof(pinStr)/sizeof(pinStr[0]);

        if(_nChars != write(_fdClose, pinStr, _nChars)) {
            close(_fdClose);
            perror(portMsg.c_str());
            exit(1);
        }
    }

    // close file to kill GPIO files
    return close(_fdClose);
#endif // __unix
}

/**
 * @brief Add new pin. It is recommended to add pins ascending
 * @param[in] pin BCM index for the selected pin
 * @param[in] direction Direction of the pin (GPIO_IN or GPIO_OUT)
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::addPin(int pin, string direction)
{
    int8_t ret = -1;
#ifdef __unix

    // GENERATE PIN ----------------------------------------

    // open executable to create GPIOs
    _fdOpen = open(_filenamePortOpen, O_WRONLY); // O_RDWR
    if(0 > _fdOpen) {
        deinit();
        perror("Error generating GPIO");
        exit(1);
    }

    string pinStr = to_string(pin);
    uint8_t strLen = (uint8_t)pinStr.length();

    // register pin in OS - pin is registered as gpioNo
    ret = write(_fdOpen, pinStr.c_str(), strLen);
    if(strLen != ret) {
        deinit();
        perror(("Error adding GPIO pin " + pinStr).c_str());
        exit(1);
    }

    // close GPIO generation executabe
    ret = close(_fdOpen);
    if(0 > ret) {
        deinit();
        perror("Error closing GPIO generation executable");
        exit(1);
    }

    // Change user permissions ----------------------------
    string tempStrPath(_pathGpio);
    string devPath = tempStrPath + "gpio" + to_string(pin) + "/direction";

    string temp = "sudo chmod 777 " + devPath;
    const char *command = temp.c_str();
    system(command);

    devPath = tempStrPath + "gpio" + to_string(pin) + "/value";
    temp = "sudo chmod 777 " + devPath;
    command = temp.c_str();
    system(command);

    devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    temp = "sudo chmod 777 " + devPath;
    command = temp.c_str();
    system(command);

    devPath = tempStrPath + "gpio" + to_string(pin) + "/edge";
    temp = "sudo chmod 777 " + devPath;
    command = temp.c_str();
    system(command);

    devPath = tempStrPath + "gpio" + to_string(pin) + "/uevent";
    temp = "sudo chmod 777 " + devPath;
    command = temp.c_str();
    system(command);

    // SET PIN DIRECTION -----------------------------------

    // set GPIO direction
    ret = setPinDir(pin, direction);
    if(0 > ret) {
        deinit();
        perror(("Error setting GPIO direction pin " + pinStr).c_str());
        exit(1);
    }
#endif // __unix

    // add pin to list
    _gpioBcm.push_back(pin);

    return ret;
}

/**
 * @brief Set operating direction of the pin
 * @param[in] pin BCM index for the selected pin
 * @param[in] direction Direction of the pin (GPIO_IN or GPIO_OUT)
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setPinDir(int pin, string direction)
{
    int8_t ret = -1;
    int fd = -1;
    uint8_t strLen = direction.length();

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/direction";

    //

    fd = open(devPath.c_str(), O_WRONLY); // O_RDWR
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, direction.c_str(), strLen);
    if(strLen != ret) {
        perror(("Error setting direction of GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " after setting direction").c_str());
        deinit();
        exit(1);
    }

    //    _pinId.push_back(fd); // add file descriptor to list for further use
#endif // __unix
    return ret;
}

/**
 * @brief Set pin active low
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setActiveLow(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_ACTIVE_LOW, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Set pin active high
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setActiveHigh(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_ACTIVE_HIGH, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Set edge rising
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setEdgeRising(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_EDGE_RISING, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Set edge falling
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setEdgeFalling(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_EDGE_FALLING, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief React to falling and rising edges
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setEdgeBoth(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_EDGE_BOTH, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief React to no edges
 * @param[in] pin BCM index for the selected pin
 * @return -1 on failure; non-negative on success
 */
int8_t
IGpio::setEdgeNone(int pin)
{
    int ret = -1;

    string pinStr = to_string(pin);

#ifdef __unix
    string tempStrPath(_pathGpio);

    string devPath = tempStrPath + "gpio" + to_string(pin) + "/active_low";
    int8_t fd = open(devPath.c_str(), O_WRONLY);
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = write(fd, GPIO_EDGE_NONE, 1);
    if(0 > ret) {
        perror(("Error setting GPIO pin " + pinStr + " active_high").c_str());
        deinit();
        exit(1);
    }

    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " active_low").c_str());
        deinit();
        exit(1);
    }
#endif // __unix
    return ret;
}

// operation methods ---------------------------------------k

/**
 * @brief Read state of the pin
 * @param[in] pin BCM index for the selected pin
 * @return Pin state: 0 = LOW, 1 = HIGH
 * @todo exclude seaching routine for pins
 */
uint8_t
IGpio::readPin(uint8_t pin)
{
    uint8_t ret = -1;
#ifdef __unix
    // find the index of the pin in the created list
    uint8_t indexPin = 0;
    for(uint8_t indPin = 0; indPin < (uint8_t)_gpioBcm.size(); indPin++) {
        if(pin == _gpioBcm.at(indPin)) {
            indexPin = indPin;
            break;
        }
    }

    // find the corresponding file descriptor
    int fd = -1; //_pinId.at(indexPin);

    string tempStrPath(_pathGpio);
    string pinStr = to_string(pin);

    // open gpio device
    string devPath = tempStrPath + "gpio" + pinStr + "/value";

    fd = open(devPath.c_str(), O_RDWR); // O_RDWR O_WRONLY
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    // read value
    int value = -1;
    ret = read(fd, &value, 1);
    if(0 > ret) {
        perror(("Error reading from GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    // close gpio device
    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " after setting direction").c_str());
        deinit();
        exit(1);
    }

    ret = value;

#endif // __unix
    return ret - 48;
}

/**
 * @brief Set pin high or low
 * @param[in] pin BCM index for the selected pin
 * @param[in] level GPIO_HIGH or GPIO_LOW
 * @return -1 on failure; non-negative on success
 * @todo Check if ved file descriptors are valid for re-opening after closing
 * @todo exclude seaching routine for pins
 */
int8_t
IGpio::writePin(uint8_t pin, string level)
{
    int8_t ret = -1;

#ifdef __unix
    // find the index of the pin in the created list
    uint8_t indexPin = 0;
    for(uint8_t indPin = 0; indPin < _gpioBcm.size(); indPin++) {
        if(pin == _gpioBcm.at(indPin)) {
            indexPin = indPin;
            break;
        }
    }

    // find the corresponding file descriptor
    int fd = -1; //_pinId.at(indexPin);

    string pinStr = to_string(pin);
    string tempStrPath(_pathGpio);

    // open gpio device
    string devPath = tempStrPath + "gpio" + pinStr + "/value";

    fd = open(devPath.c_str(), O_WRONLY); // O_RDWR
    if(0 > fd) {
        perror(("Error opening GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    // set value
    ret = write(fd, level.c_str(), 1);
    if(0 > ret) {
        perror(("Error writing to GPIO pin " + pinStr).c_str());
        deinit();
        exit(1);
    }

    // close gpio device
    ret = close(fd);
    if(0 > ret) {
        perror(("Error closing GPIO pin " + pinStr + " after setting direction").c_str());
        deinit();
        exit(1);
    }

#endif // __unix

    return ret;
}

// =========================================================
// II2c_Master
// =========================================================

// constructors and destructors ----------------------------

/**
 * @brief Constructor
 */
II2c_Master::II2c_Master(void){}

/**
 * @brief Destructor
 */
II2c_Master::~II2c_Master(void) {}

// configuration methods -----------------------------------

#ifdef __unix
/**
 * @brief Specify the device to be used
 * @param[in] filename File specifying the device for the interface
 */
void
II2c_Master::setDevice(char *filename)
{
    _filenameDevice = filename;
}
#endif // __unix

/**
 * @brief Set the address from the device
 * @param[in] add Address of the device
 */
void
II2c_Master::setAddr(int add)
{
    _addr = add;
}

// operating methods ---------------------------------------


/**
 * @brief Opens the device
 * @return -1 on failure; non-negative on success
 */
int8_t
II2c_Master::openDevice(void)
{
#ifdef __unix
    string temp = "sudo chmod 777 " + std::string(_filenameDevice);
    const char *command = temp.c_str();
    system(command);

    _fileId = open(_filenameDevice, O_RDWR);
    if (_fileId < 0) {
        closeDevice();
        perror("Error opening I2C device");
        exit(1);
    }
#endif // __unix

    return _fileId;
}

/**
 * @brief Closes the device
 * @return -1 on failure; non-negative on success
 */
int8_t
II2c_Master::closeDevice(void)
{
    int8_t ret = -1;
#ifdef __unix
    ret = close(_fileId);
    if (_fileId < 0) {
        perror("Error clossing I2C device");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Send the device address to the BUS
 * @pre II2c_Master::setAddr() has to be called first
 * @return -1 on failure; 0 on success
 */
int8_t
II2c_Master::sendAddr(void)
{
    int8_t ret = -1;
#ifdef __unix
    ret = ioctl(_fileId, I2C_SLAVE, _addr);
    if (ret < 0) {
        closeDevice();
        perror("Error sensing I2C address");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Send the device address to the BUS
 * @param[in] addr Address to be send
 * @return -1 on failure; 0 on success
 */
int8_t
II2c_Master::sendAddr(uint8_t addr)
{
    int8_t ret = -1;
#ifdef __unix
    ret = ioctl(_fileId, I2C_SLAVE, addr);
    if (ret < 0) {
        closeDevice();
        perror("Error sensing I2C address");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Receive data from the BUS
 * @param[out] dataRx Field to store data in
 * @param[in] length Number of bytes to receive
 * @return -1 on failure; 0 on success
 */
int8_t
II2c_Master::readBytes(uint8_t *dataRx, uint8_t length)
{
    int8_t ret = -1;
#ifdef __unix
    ret = read(_fileId, dataRx, length);
    if (ret != length) {
        //read() returns the number of bytes actually read,
        // if it doesn't match then an error occurred
        // (e.g. no response from the device)
        closeDevice();
        perror("Error reading data");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Send data to the BUS
 * @param[in] dataTx Field with data to send
 * @param[in] length Number of bytes to send
 * @return -1 on failure; 0 on success
 */
int8_t
II2c_Master::writeBytes(uint8_t *dataTx, uint8_t length)
{
    int8_t ret = -1;
#ifdef __unix
    ret = write(_fileId, dataTx, length);
    if (ret != length) {
        //read() returns the number of bytes actually read,
        // if it doesn't match then an error occurred
        // (e.g. no response from the device)
        closeDevice();
        perror("Error writing data");
        exit(1);
    }
#endif // __unix

    return ret;
}

// =========================================================
// ISpi_Master
// =========================================================

// constructors and destructors ----------------------------

/**
 * @brief Constructor
 */
ISpi_Master::ISpi_Master(void)
{
#ifdef __unix
    //
#endif // __unix
}

/**
 * @brief Destructor
 */
ISpi_Master::~ISpi_Master(void)
{
#ifdef __unix
    //
#endif // __unix
}

// operating methods ---------------------------------------

/**
 * @brief Specify the device to be used and initializes the interface and opens the device
 * @param[in] filename File specifying the device for the interface
 * @param[in] mode (CPOL, CPHA): 0 = (0, 0), 1 = (0, 1), 2 = (1, 0), 3 = (1, 1) CPOL indicates the initial clock polarity. CPOL = 0 means the clock starts low, so the first (leading) edge is rising, and the second (trailing) edge is falling. CPOL = 1 means the clock starts high, so the first (leading) edge is falling. CPHA indicates the clock phase used to sample data; CPHA = 0 says sample on the leading edge, CPHA = 1 means the trailing edge.
 * @param[in] speed Clock speed [Hz]
 * @param[in] bits Bits per word
 * @param[in] bitorder 0: MSB first, 1: LSB first
 * @param[in] csRelease 0 = Set CS high after a transfer, 1 = leave CS set low after transmission
 * @return -1 on failure; non-negative on success
 */
int8_t
ISpi_Master::openDevice(char *filename, int mode, int speed, int bits, int bitorder, int csRelease)
{
    int ret = -1;
#ifdef __unix
    // define the needed parameters
    _filenameDevice = filename;

    string temp = "sudo chmod 777 " + std::string(_filenameDevice);
    const char *command = temp.c_str();
    system(command);

    switch(mode) {
    case 0:
        _mode = SPI_MODE_0;
        break;
    case 1:
        _mode = SPI_MODE_1;
        break;
    case 2:
        _mode = SPI_MODE_2;
        break;
    case 3:
        _mode = SPI_MODE_3;
        break;
    default:
        break;
    }

    _speed = speed;
    _bits = bits;
    _bitorder = bitorder;

    memset(&_spiObj, 0, sizeof(spi_ioc_transfer));
    _spiObj.delay_usecs = 0;
    _spiObj.speed_hz = _speed;
    _spiObj.bits_per_word = _bits;

    _fileId = open(_filenameDevice, O_RDWR);
    if (_fileId < 0) {
        perror("Error opening SPI device");
        exit(1);
    }

    // initialize interface
    ret = ioctl(_fileId, SPI_IOC_WR_MODE, &_mode); // set mode
    if(ret < 0) {
        closeDevice();
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    ret = ioctl(_fileId, SPI_IOC_WR_LSB_FIRST, &_bitorder); // set MSB/LSB first
    if(ret < 0) {
        closeDevice();
        perror("Could not set LSB/MSB (WR)...ioctl fail");
        exit(1);
    }

    ret = ioctl(_fileId, SPI_IOC_WR_BITS_PER_WORD, &_bits); // set wordlength
    if(ret < 0) {
        closeDevice();
        perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
        exit(1);
    }

    ret = ioctl(_fileId, SPI_IOC_WR_MAX_SPEED_HZ, &_speed); // set clock speed
    if(ret < 0) {
        closeDevice();
        perror("Could not set SPI speed (WR)...ioctl fail");
        exit(1);
    }

    _csRelease = csRelease;
#endif // __unix
    return _fileId;
}

/**
 * @brief Closes the device
 * @return -1 on failure; non-negative on success
 */
int8_t
ISpi_Master::closeDevice(void)
{
    int8_t ret = -1;
#ifdef __unix
    ret = close(_fileId);
    if (ret < 0) {
        perror("Error closing SPI device");
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Reads and writes data simultanously from/to the BUS. The data of dataTx are written to the BUS whereas the data of dataRx are read from the BUS at the same time.
 * @param[in] dataRx Field to store data in
 * @param[out] dataTx Field with data to send
 * @param[in] lenRxTx Number of bytes to receive and send
 * @return -1 on failure; number of read bytes on success
 */
int8_t
ISpi_Master::readWriteBytes(uint8_t *dataRx, uint8_t *dataTx, uint8_t lenRxTx)
{
    int8_t ret = -1;
#ifdef __unix
    // write parameters to SPI object
    _spiObj.rx_buf = (unsigned long)dataRx;
    _spiObj.tx_buf = (unsigned long)dataTx;
    _spiObj.len = lenRxTx;
    _spiObj.cs_change = _csRelease;

    // transfering data
    // the argument in SPI_IOC_MESSAGE seems to be the dimension of _spiObj. Not sure...
    ret = ioctl(_fileId, SPI_IOC_MESSAGE(1), &_spiObj);
    if(0 > ret) {
        perror("Error - Problem transmitting spi data..ioctl");
        closeDevice();
        exit(0);
    }
#endif // __unix
    return ret;
}

// =========================================================
// ITcpClient
// =========================================================

// constructors and destructors ----------------------------

/**
 * @brief Constructor
 */
ITcpClient::ITcpClient(void)
{
#ifdef __unix
    //
#endif // __unix
}

/**
 * @brief Destructor
 */
ITcpClient::~ITcpClient(void)
{
#ifdef __unix
    //
#endif // __unix
}

/**
 * @brief Setup the Client configuration, parse the port number
 * @param[in] serverIp IP address of the server
 * @param[in] pNum Number of the port. It is recommended to use only values above 2000.
 */
void
ITcpClient::init(char* serverIp, int pNum)
{
    // set port number and IP adress of the server to connect
    _serverIp = serverIp;
    _portNum = pNum;

#ifdef __unix
    _fileId = socket(AF_INET, SOCK_STREAM, 0);
    if(0 > _fileId) {
        closeSocket();
        perror("Error creating client socket");
        exit(1);
    }

    // setup a socket and connection tools
    _server = gethostbyname(_serverIp);
    if(NULL == _server) {
        closeSocket();
        cout << "Error finding server: No such host" << endl;
        exit(1);
    }

    // copy the IP from _server to _serverAddr
    bzero((char*) &_serverAddr, sizeof(_serverAddr));

    // configure server struct ------------------------------------------
    _serverAddr.sin_family = AF_INET; // set address type
    _serverAddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr*)*_server->h_addr_list));
    _serverAddr.sin_port = htons(_portNum); // set port number
#endif // __unix
}

/**
 * @brief Establish a connection to the server
 * @return -1 on failure; 0 on success
 */
int8_t
ITcpClient::connect2server(void)
{
    int8_t ret = -1;

#ifdef __unix
    // https://www.linuxhowtos.org/data/6/connect.txt
    ret = connect(_fileId, (struct sockaddr*) &_serverAddr, sizeof(_serverAddr));
    if(0 > ret) {
        closeSocket();
        perror("Error client when connecting to server");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Checks if the Server on the other side is still connected.
 * @return -1 on failure, >= 0 otherwise
 * @post closeSocket() should be called.
 * @todo test
 */
int8_t
ITcpClient::isConnected(void)
{
    // https://groups.io/g/twsapi/topic/4046052
    int8_t ret = -1;

#ifdef __unix
    uint8_t bytesAvailable = 0;

    //ret = FD_ISSET(_socketAccept, read_fd_set);
    if(!ret) {
        cout << "Other socket disconnected" << endl;
    }
    //ret = ioctl(_socketAccept, FIONREAD, &bytesAvailable);
#endif // __unix
    return ret;
}

/**
 * @brief Read data from the server. This methods returns max. 255 characters. The input buffer has to be cleared before.
 * @param[out] Rx Data read from the server
 * @return -1 on failure; number of received characters on success
 */
int
ITcpClient::receive(char *Rx)
{
    _nChars = -1;

    memset(&_bufRx ,0, sizeof(_bufRx)); // clear buffer

#ifdef __unix
    _nChars = recv(_fileId, (char*)&_bufRx, strlen(_bufTx), 0);
    if(0 > _nChars) {
        closeSocket();
        perror("Error client when reading from server");
        exit(1);
    }
#endif // __unix

    strcpy(Rx, _bufRx);

    return _nChars;
}

/**
 * @brief Write data to the server. This methods transmits max. 255 characters.
 * @param[in] Tx Data to write
 * @param[in] nByte Number of characters to write
 * @return -1 on failure; 0 on success
 */
int
ITcpClient::transmit(char *Tx, int nByte)
{
    _nChars = -1;

    memset(&_bufTx ,0, sizeof(_bufTx)); // clear buffer
    strcpy(_bufTx, Tx); // copy data in transmision buffer

#ifdef __unix
    _nChars = send(_fileId, (char*)&_bufTx, strlen(_bufTx), 0);
    if(0 > _nChars) {
        closeSocket();
        perror("Error client when writing to server");
        exit(1);
    }
#endif // __unix

    return _nChars;
}

/**
 * @brief Closes the port
 * @return -1 on failure; 0 on success
 */
int8_t
ITcpClient::closeSocket(void)
{
    int8_t ret = -1;

#ifdef __unix
    ret = close(_fileId);
#endif // __unix

    return ret;
}

// =========================================================
// ITcpServer
// =========================================================

/**
 * @brief Constructor
 */
ITcpServer::ITcpServer(void)
{
#ifdef __unix
    //
#endif // __unix
}

/**
 * @brief Destructor
 */
ITcpServer::~ITcpServer(void)
{
    //
}

/**
 * @brief Setup the Server configuration, parse the port number and bind the socket to the address and the port number.
 * @param[in] pNum Number of the port. It is recommended to use only values above 2000.
 */
void
ITcpServer::init(int pNum)
{
    // set port number
    _portNum = pNum;

#ifdef __unix
    /* create new socket
     * arguments:
     * - Address domain of the socket (AF_INET for internet (latter) and AF_UNIX for unix domain (former))
     * - Type of socket (SOCK_STREAM and SOCK_DGRAM)
     * - Protocol: 0 is recommended so the system choses TCP or UDP for the mentioned socket type
     *
     * https://pubs.opengroup.org/onlinepubs/007908775/xns/socket.html
     * https://www.linuxhowtos.org/data/6/socket.txt
     * https://www.linuxhowtos.org/C_C++/socket.htm
     */
    _fileId = socket(AF_INET, SOCK_STREAM, 0);
    if(0 > _fileId) {
        closeSocket();
        perror("Error creating server socket");
        exit(1);
    }

    // set all values to zero
    bzero((char*) &_servAddr, sizeof(_servAddr));

    // populate the struct
    _servAddr.sin_family = AF_INET; // should be done this way
    _servAddr.sin_port = htons(_portNum); // convert int to network byte order (host byte order)
    _servAddr.sin_addr.s_addr = htonl(INADDR_ANY); // IP of the host (machine which runs the server) which is determined here

    // Connect the socket to the address of the server and the port number
    // https://www.linuxhowtos.org/data/6/bind.txt
    int8_t ret = bind(_fileId, (struct sockaddr*) &_servAddr, sizeof(_servAddr));
    if(0 > ret) {
        closeSocket();
        perror("Error on binding server");
        exit(1);
    }

    _clilen = sizeof(_cliAddr);
#endif // __unix
}

/**
 * @brief Closes the port
 * @return -1 on failure; 0 on success
 */
int8_t
ITcpServer::closeSocket(void)
{
    int8_t ret = -1;
#ifdef __unix
    // https://www.linuxhowtos.org/manpages/3p/close.htm
    ret = close(_socketAccept);
    if(0 > ret) {
        perror("Error server disconnecting client");
        exit(1);
    }

    ret = close(_fileId);
    if(0 > ret) {
        perror("Error server closing socket");
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Checks if the Client on the other side is still connected.
 * @return -1 on failure, >= 0 otherwise
 * @post closeSocket() should be called.
 * @todo test
 */
int8_t
ITcpServer::isConnected(void)
{
    // https://groups.io/g/twsapi/topic/4046052
    int8_t ret = -1;

#ifdef __unix
    uint8_t bytesAvailable = 0;

    //ret = FD_ISSET(_socketAccept, read_fd_set);
    if(!ret) {
        cout << "Other socket disconnected" << endl;
    }
    //ret = ioctl(_socketAccept, FIONREAD, &bytesAvailable);
#endif // __unix
    return ret;
}

/**
 * @brief Waits for a client to connect
 * @pre init() has to be called first.
 * @return -1 on failure; 0 on success
 */
int8_t
ITcpServer::wait4client(void)
{
    cout << "Waiting for client..." << endl;

    int8_t ret = -1;

#ifdef __unix
    // https://www.linuxhowtos.org/data/6/listen.txt
    ret = listen(_fileId, 5); // wait for connection (max. 5 at a time permitted on most systems)
    if(0 > ret) {
        closeSocket();
        perror("Error server when listening");
        exit(1);
    }
#endif // __unix

    return ret;
}

/**
 * @brief Accept the client
 * @pre wait4client() has to be called first
 * @post Client is successfully connected to the server.
 * @return -1 on failure; 0 on success
 */
int
ITcpServer::acceptClient(void)
{
#ifdef __unix
    /* block until a client connects to the server.
     * When a connection has been established sucessfully the process wakes up.
     * https://www.linuxhowtos.org/data/6/accept.txt */
    _socketAccept = accept(_fileId, (struct sockaddr*) &_cliAddr, &_clilen);
    if(0 > _socketAccept) {
        closeSocket();
        perror("Error server when accepting client");
        exit(1);
    }
#endif // __unix

    return _socketAccept;
}

/**
 * @brief Read data from the client. This methods returns max. 255 characters. The input buffer has to be cleared before.
 * @param[out] Rx Buffer containing the read characters.
 * @pre acceptClient() has to be called first
 * @return -1 on failure; number of received characters on success
 */
int
ITcpServer::receive(char *Rx)
{
    _nChars = -1;

    //    bzero(Rx, sizeof(Rx)); // initialize and clear the buffer
    //    memset(Rx, 0, sizeof(Rx));
    memset(&_bufRx, 0, sizeof(_bufRx));

#ifdef __unix
    /* reading data from the client.
     * read() blocks until there is something to read in the socket after the client executed write().
     * https://www.linuxhowtos.org/data/6/read.txt */
    //_nChars = read(_socketAccept, _bufRx, 255);
    _nChars= recv(_socketAccept, (char*)&_bufRx, sizeof(_bufRx), 0);
    if(0 > _nChars) {
        closeSocket();
        perror("Error server when reading from client");
        exit(1);
    }
#endif // __unix
    strcpy(Rx, _bufRx); // copy to extern variable

    return _nChars;
}

/**
 * @brief Writing data to the client.
 * @param[in] Tx Data to write
 * @param[in] nByte Number of characters to write. This methods transmits max. 255 characters.
 * @return Number of written characters on success or -1 on failure
 */
int
ITcpServer::transmit(char *Tx, int nByte)
{
    _nChars = -1;

    // write to intern buffer
    //    for(int indTx = 0; indTx < nByte; indTx++) {
    //        _bufTx[indTx] = Tx[indTx];
    //    }
    memset(&_bufTx, 0, sizeof(_bufTx));
    strcpy(_bufTx, Tx);

#ifdef __unix
    /* write data to the client
     * https://www.linuxhowtos.org/data/6/write.txt */
    //    _nChars = write(_socketAccept, Tx, nByte);
    _nChars = send(_socketAccept, (char*)&_bufTx, strlen(_bufTx), 0);
    if(0 > _nChars) {
        closeSocket();
        perror("Error server when writing to client");
        exit(1);
    }
#endif // __unix

    return _nChars;
}

// =========================================================
// IUart
// =========================================================

/**
 * @brief Constructor
 */
IUart::IUart(void)
{
#ifdef __unix
#endif // __unix
}

/**
 * @brief Destructor
 */
IUart::~IUart(void)
{
#ifdef __unix
#endif // __unix
}

#ifdef __unix
/**
 * @brief Initialize the interface
 * @param[in] filename File specifying the device for the interface
 * @param[in] baud Baud rate
 */
void
IUart::openDevice(char *filename, int baud)
{
    // open device
    //_fileId = open(filename, O_RDWR);
    _fileId = open("filename", O_RDWR | O_NOCTTY | O_NDELAY);
    if(!_fileId) {
        closeDevice();
        perror("Error opening UART");
        exit(1);
    }
}
#endif // __unix

/**
 * @brief Closes the device
 * @return -1 on failure; non-negative on success
 */
int8_t
IUart::closeDevice(void)
{
    int8_t ret = -1;
#ifdef __unix
    ret = close(_fileId);
    if(!ret) {
        perror("Error closing UART");
        exit(1);
    }
#endif // __unix
    return ret;
}

/**
 * @brief Receive data from the bus. This methods transmits max. 255 characters.
 * @param[out] Rx Buffer containing the read characters.
 * @return -1 on failure; number of received characters on success
 */
int
IUart::receive(int32_t *Rx)
{
    int nBytes = 0;
#ifdef __unix
    nBytes = ioctl(_fileId, _IOR('a', 'b', int32_t*), Rx);
#endif // __unix
    return nBytes;
}

/**
 * @brief Write data to the bus. This methods transmits max. 255 characters.
 * @param[in] Tx Data to write
 * @param[in] nByte Number of characters to write
 * @return Number of transmitted bytes
 */
int
IUart::transmit(int32_t *Tx, int nByte)
{
    int nBytes = 0;
#ifdef __unix
    nBytes = ioctl(_fileId, _IOW('a', 'a', int32_t*), Tx);
    if(!nByte) {
        perror("Error transmitting via UART");
        exit(0);
    }
#endif // __unix
    return nBytes;
}
