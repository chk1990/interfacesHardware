/**
 * @author Christoph Kolhoff
 * @file mcp2515.cpp
 */

#include <iostream>

#include "interfacesHardware.h"
#include "mcp2515.h"

using namespace std;

/**
 * @brief Constructor
 */
Mcp2515::Mcp2515(void)
{
#ifdef __unix

#endif // __unix
}

/**
 * @brief Destructor
 */
Mcp2515::~Mcp2515(void)
{
#ifdef __unix

#endif // __unix
}

/**
 * @brief Initializes the SPI interface to be used with the MCP2515 controller. By default all interrupts are enabled, all types of identifiers are received and rollover mode is enabled.
 *
 * @param[in] filename File specifying the SPI-device for the interface
 * @param[in] mode (CPOL, CPHA): 0 = (0, 0), 1 = (1, 1) CPOL indicates the initial clock polarity. CPOL = 0 means the clock starts low, so the first (leading) edge is rising, and the second (trailing) edge is falling. CPOL = 1 means the clock starts high, so the first (leading) edge is falling. CPHA indicates the clock phase used to sample data; CPHA = 0 says sample on the leading edge, CPHA = 1 means the trailing edge.
 * @param[in] speed Clock speed [Hz]
 * @return -1 on failure; non-negative on success
 * @post Device is in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
int8_t
Mcp2515::init(char* filename, int mode, int speed)
{
    int8_t ret = -1;
#ifdef __unix
    ret = _interface->openDevice(filename, mode, speed, 8, 0, 0);
#endif // __unix

    // reset all configurations of the device
    resetDevice();
    usleep(10000); // wait for 10 ms

    //setModeConfig();

    // set pin interrupts
    uint8_t regInterr[2] = {0xFF, 0};
    writeReg(0x2B, regInterr, 1); // CANINTE

    regInterr[0] = 0;
    writeReg(0x2C, regInterr, 1); // CANINTF
    modBit(0x2D, 0b11000000, 0); // clear interrupt flags for receive
    
    uint8_t valBFPCTRL = 0b00001111;
    writeReg(0x0C, &valBFPCTRL, 1); // BFPCTRL
    
    uint8_t valTXRTSCTRL = 0b00000111;
    //writeReg(0x0D, &valTXRTSCTRL, 1); // TXRTSCTRL
    modBit(0x0D, 0b00000111, valTXRTSCTRL);
    
    // declare message types to receive
    configRxBuf(0, 3);
    configRxBuf(1, 3);

    //enableRollover();

    //clearErrors();

    return ret;
}

/**
 * @brief Reset all present configurations of the device
 * @post Device is in Configuration Mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::resetDevice(void)
{
    uint8_t byteIn = 0xC0;
    uint8_t byteOut = 0;

#ifdef __unix
    _interface->readWriteBytes(&byteOut, &byteIn, 1);
#endif // __unix

    usleep(10000); // pass time to reset
}

/**
 * @brief Get the ID type of the received mesage.
 * @param[in] indRxb Index of the transmission buffer (0-1). The input value has to be the returned value of getInterruptFlags & 0x03.
 * @return 0: Standard Frame, 1: Extended frame
 * @pre check4message() has to return a valid result
 * @attention See clearErrors()
 */
uint8_t
Mcp2515::getIdentifierType(uint8_t indRxb)
{
    // uint8_t indRxb = getInterruptFlags();

    uint8_t dataReg[2] = {};

    readReg(0x62 + 16 * indRxb, dataReg, 2);

    return ((dataReg[1] & 8) >> 3);
}

/**
 * @brief Set the standard identifier for the receiving device and send it to the device
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @param[in] identType Type of identifier - 0: Standard, 1: Extended
 * @return Identifier value
 */
uint32_t
Mcp2515::getIdentifierRx(uint8_t indRxb, uint8_t identType)
{
    uint32_t ident = 0;
    uint8_t dataReg[3] = {};

    if(!identType) {
        // standard
        readReg(0x61 + 16 * indRxb, dataReg, 2);

        // fuse to uint_32
        ident = (uint32_t)(((uint32_t)dataReg[0]) << 3) | (dataReg[1] >> 5);

        //printf("Identifier bytes: %u %u\n\r", dataReg[0], dataReg[1]);
    }
    else if(1 == identType) {
        // extended
        readReg(0x62 + 16 * indRxb, dataReg, 3);

        // fuse to uint_32
        ident = (uint32_t)((((uint32_t)dataReg[0] & 3) << 16) | ((uint32_t)dataReg[1] << 8) | (dataReg[2]));

        //printf("Identifier bytes: %u %u %u\n\r", dataReg[0], dataReg[1], dataReg[2]);
    }

    return ident;
}

/**
 * @brief Configure the Rx buffer - Choose which types of identifiers are considered.
 * @param[in] indRxb indRxb Index of the transmission buffer (0-1)
 * @param[in] modeRx Operation mode of the buffer (0: Standard and extended frames, 1: Standard frames only, 2: Extended filters only, 3: All messages)
 * @pre init() has to be called first
 */
void
Mcp2515::configRxBuf(uint8_t indRxb, uint8_t modeRx)
{
    uint8_t regVal = 0;
    
    switch(modeRx) {
    case 0:
        regVal = 0b00000000;
        break;
    case 1:
        regVal = 0b00100000;
        break;
    case 2:
        regVal = 0b01000000;
        break;
    case 3:
        regVal = 0b01100000;
        break;
    }

    //writeReg(0x60 + 16 * indRxb, &regVal, 1);
    modBit(0x60 + 16 * indRxb, 0b01100000, regVal);
    //modBit(0x70, 0b01100000, regVal);
}

/**
 * @brief Set the mask and filter bits to apply identifier bits to the received message
 * @param[in] indFiltRxb Index of the receiving filter (0-1 for RXB0 and 0-5 for RXB1)
 * @param[in] identVal Value of the identifier
 * @param[in] identType Type of identifier - 0: Standard, 1: Extended, 2: Any - no masking - rollover enabled, 3: Standard and Extended meeting filter criteria
 * @param[in] indRxb Index of the receive buffer (0-1)
 * @pre init() has to be called first
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 * @attention Do not use disableRollover() or enableRollover() before or after this method. These can only be used when turning off the masking.
 */
void
Mcp2515::setFilterRx(uint8_t indFiltRxb, uint32_t identVal, uint8_t identType, uint8_t indRxb)
{    
    // declare which filter should be used
    uint8_t regVal = 0;
    uint8_t data[8] = {};

    // set masking bytes and identifier values
    if(0 == identType) {
        parseIdentStd(data, identVal); // identifier in first two registers, ignore value of extended identifier

        data[4] = 0xFF; // masking bytes
        data[5] = 0b11100000;
        data[6] = 0;
        data[7] = 0;
    }
    else if(1 == identType) {
        parseIdentExt(data, identVal);

        data[4] = 0; // masking bytes
        data[5] = 0b00000011;
        data[6] = 0xFF;
        data[7] = 0xFF;
    }

    writeReg(4 * indFiltRxb, data, 4);
    writeReg(0x20 + indRxb * 4, data + 4, 4);

    // specify identifier type - Bits RXM0/1
    switch(identType) {
    case 0: // standard
        disableRollover();
        regVal = 0b00100000;
        break;
    case 1: // extended
        disableRollover();
        regVal = 0b01000000;
        break;
    case 2: // any
        enableRollover();
        regVal = 0b01100000;
        break;
    default:
        break;
    }

    // enable BUKT
    if((1 == indRxb) & (1 <= indFiltRxb)) {
        regVal = regVal | 0b00000100;
    }

    // specify filter index
    regVal = regVal | indFiltRxb;

    // send information
    writeReg(0x60 + 0x10 * indRxb, &regVal, 1);
}

/**
 * @brief Shutdown the device and disconect device
 * @return -1 on failure; non-negative on success
 * @attention This method has to be called when the interface is not longer needed.
 */
int8_t
Mcp2515::terminate(void)
{
    terminateTx();
    int8_t ret = -1;
#ifdef __unix
    ret = _interface->closeDevice();
#endif // __unix
    return ret;
}

/**
 * @brief Reads the value of a given register
 * @param[in] reg Address of the register to start reading from
 * @param[out] dataReg Array to store read data in
 * @param[in] nBytes Number of bytes to read (1-253)
 */
void
Mcp2515::readReg(uint8_t reg, uint8_t* dataReg, uint8_t nBytes)
{
    int8_t ret = -1;

    uint8_t dataIn[255] = {};
    dataIn[0] = 3; // READ
    dataIn[1] = reg; // reg

    uint8_t dataOut[255] = {};

#ifdef __unix
    ret = _interface->readWriteBytes(dataOut, dataIn, nBytes + 2);
    if(0 > ret) {
        printf("Failed to read MCP2515 register %u\n\r", dataIn[1]);
        terminate();
    }
#endif // __unix

    // store read values in external array
    for(uint8_t indByte = 0; indByte < nBytes; indByte++) {
        *(dataReg + indByte) = dataOut[indByte + 2];
    }
}

/**
 * @brief Reset the value to a given register
 * @param[in] reg Address of the register to reset
 * @post setModeNormal() or similar has to be called afterwards.
 * @return -1 on failure; non-negative on success
 */
int8_t
Mcp2515::resetReg(uint8_t reg)
{
    int8_t ret = -1;

    uint8_t dataIn[3] = {192}; // RESET
    dataIn[1] = reg; // address

    uint8_t dataOut[3] = {0, 0, 0};

#ifdef __unix
    ret = _interface->readWriteBytes(dataOut, dataIn, 3);
    if(0 > ret) {
        printf("Failed to write to MCP2515 register %u\n\r", dataIn[1]);
        terminate();
    }

    //sleep(1); // wait for reset to be complete
#endif // __unix

    return ret;
}

/**
 * @brief Terminate all transmission processes
 */
void
Mcp2515::terminateTx(void)
{
    uint8_t value = 0b00010000;
    writeReg(0x0F, &value, 1);

    modBit(0x30, 0b00001000, 0);
    modBit(0x40, 0b00001000, 0);
    modBit(0x50, 0b00001000, 0);
}

/**
 * @brief Writes the value to a given register
 * @param[in] reg Address of the register to read from
 * @param[in] val Value to be set
 * @param[in] nBytes Length of val (1-253)
 */
void
Mcp2515::writeReg(uint8_t reg, uint8_t* val, uint8_t nBytes)
{
    int8_t ret = -1;

    uint8_t dataIn[255] = {};
    dataIn[0] = 2; // WRITE
    dataIn[1] = reg; // address to start

    // parse values to array to be send
    for(uint8_t indByte = 0; indByte < nBytes; indByte++) {
        dataIn[indByte + 2] = *(val + indByte);
    }

    uint8_t dataOut[255] = {};

#ifdef __unix
    ret = _interface->readWriteBytes(dataOut, dataIn, nBytes + 2);
    if(0 > ret) {
        printf("Failed to write to MCP2515 register %u\n\r", dataIn[1]);
        terminate();
    }
#endif // __unix
}

/**
 * @brief Declare a message as ready to send
 * @param[in] indTxb Index of the transmit buffer (0-2)
 * @return -1 on failure; non-negative on success
 */
int8_t
Mcp2515::setRts(uint8_t indTxb)
{
    int8_t ret = -1;

    uint8_t dataOut = 0b10000000; // initial command
    uint8_t dataSend = dataOut | (1 << (indTxb)); // create byte to send

#ifdef __unix
    ret = _interface->readWriteBytes(&dataOut, &dataSend, 1);
    if(0 > ret) {
        printf("Failed to set MCP2515 register RTS for buffer %u\n\r", indTxb);
        terminate();
    }
#endif // __unix

    return ret;
}

/**
 * @brief Read data that has been received from the BUS
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @param[out] data Data received
 * @return Number of received bytes
 */
uint8_t
Mcp2515::receiveData(uint8_t indRxb, uint8_t* data)
{
    uint8_t nBytes = getNumBytesReceived(indRxb);
    
    readReg(0x66 + indRxb * 16, data, nBytes);

    // clear interrupt flag
    uint8_t maskByte = 0b11100000 | (1 << indRxb);
    uint8_t valueByte = 0x00;
    modBit(0x2C, maskByte, valueByte); // RX buffer full flags
    modBit(0x2D, (1 << (6 + indRxb)), valueByte); // overflow flags

    return nBytes;
}

/**
 * @brief Transmit data on the BUS. Data is send when the BUS is free.
 * @param[in] identVal Identifier to transmit
 * @param[in] identType Type of the identifier (0: standard, 1: extended)
 * @param[in] dataIn Data to be transmitted
 * @param[in] nBytes Length of dataIn (1-8)
 * @param[in] indTxb Index of the transmission buffer (0-2)
 * @param[in] prio Priority (0: lowest - 3: highest)
 * @return -1 on failure; non-negative on success
 * @pre Device has to be in normal mode
 */
int8_t
Mcp2515::transmitData(uint32_t identVal, uint8_t identType, uint8_t* dataIn, uint8_t nBytes, uint8_t indTxb, uint8_t prio)
{
    int8_t ret = -1;

    setModeNormal();

    modBit(0x30 + 16 * indTxb, 0b00001011, prio);

    /* buffer to transmit data to controller
     * mapping:
     * [0] LOAD instruction for selected buffer
     * [1] TXBnSIDH
     * [2] TXBnSIDL
     * [3] TXBnEID8
     * [4] TXBnEID0
     * [5] TXBnDLC - Number of data bytes to be transfered
     * [6 - 13] TXBnDm - Data bytes
     */
    uint8_t bufTx[14] = {};
    bufTx[0] = 0x40 | (2 * indTxb);

    // set identifier
    if(0 == identType) {
        // standard
        parseIdentStd(bufTx + 1, identVal);
    }
    else if(1 == identType) {
        // extended
        //        dataReg[1] = 0x33 + 16 * indTxb; // register to start
        parseIdentExt(bufTx + 1, identVal);
    }

    // set number of data bytes
    bufTx[5] = nBytes;

    // parse data bytes to buffer
    for(uint8_t indByte = 0; indByte < nBytes; indByte++) {
        bufTx[6 + indByte] = *(dataIn + indByte);
    }

    uint8_t dataOut[14] = {};

    // send entire buffer
#ifdef __unix

    ret = _interface->readWriteBytes(dataOut, bufTx, 14);
    if(0 > ret) {
        printf("Failed to write to MCP2515 register %u\n\r", dataIn[1]);
        terminate();
    }
#endif // __unix

    // ready to send if BUS is free
    setRts(indTxb);

    return ret;
}

/**
 * @brief Convert the standard identifier value to bytes that can be send
 * @param[out] data Array that is send afterward. The address of the WRITE instruction has to be provided.
 * @param[in] value Identifier value
 */
void
Mcp2515::parseIdentStd(uint8_t* data, uint32_t value)
{    
    // convert input values to byte order
    *(data) = (value >> 3) & 0xFF; // TXBnSIDH
    *(data + 1) = (value << 5) & 0xFF; // TXBnSIDL
    *(data + 2) = 0; // TXBnEID8 - don't know why
    *(data + 3) = 0; // TXBnEID0 - don't know why
}

/**
 * @brief Convert the extended identifier value to bytes that can be send
 * @param[out] data Array that is send afterward. The address of the WRITE instruction has to be provided.
 * @param[in] value Identifier value
 * @todo test
 */
void
Mcp2515::parseIdentExt(uint8_t* data, uint32_t value)
{
    // convert input values to byte order
    *data = 0xFF; // TXBnSIDH
    *(data + 1) = 0xE8 | ((value >> 16) & 0xFF); // TXBnSIDL
    *(data + 2) = (value >> 8) & 0xFF; // TXBnEID8
    *(data + 3) = value & 0xFF; // TXBnEID0
}

/**
 * @brief Set the TXBnDLC register for the receiving device and send it to the device (Data Length Code)
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @return Number of received bytes (0-8)
 */
uint8_t
Mcp2515::getDataLenCodeRx(uint8_t indRxb)
{
    uint8_t dataLengthCode = 0;

    readReg(0x65 + indRxb * 16, &dataLengthCode, 1);

    return dataLengthCode;
}

/**
 * @brief Modify selected bits of a register
 * @param[in] addr Address of the register
 * @param[in] mask Declares which bits have to be modified
 * @param[in] data Provides the new values
 */
void
Mcp2515::modBit(uint8_t addr, uint8_t mask, uint8_t data)
{
#ifdef __unix
    uint8_t dataIn[4] = {};
    uint8_t dataOut[4];
    dataIn[0] = 5; // Bit modify
    dataIn[1] = addr;
    dataIn[2] = mask;
    dataIn[3] = data;

    _interface->readWriteBytes(dataOut, dataIn, 4);
#endif // __unix
}

/**
 * @brief Sets the mode of the controller
 * @param mode 0: Normal Operation Mode, 1: Sleep Mode, 2: Loopback Mode, 3: Listen Only Mode, 4: Configuration Mode
 */
void
Mcp2515::setMode(uint8_t mode)
{
    modBit(0x0F, 0b11100000, mode << 5); // CANCTRL
}

/**
 * @brief Sets the controller into Normal mode. This mode is required to operate as assumed by the user.
 */
void
Mcp2515::setModeNormal(void)
{
    setMode(0);
}

/**
 * @brief Sets the controller into Sleep mode
 */
void
Mcp2515::setModeSleep(void)
{
    setMode(1);
}

/**
 * @brief Sets the controller into Listen mode. For initialization etc.
 */
void
Mcp2515::setModeListen(void)
{
    setMode(2);
}

/**
 * @brief Sets the controller into Loopback mode. For development only.
 */
void
Mcp2515::setModeLoopback(void)
{
    setMode(3);
}

/**
 * @brief Sets the controller into Configuration mode. For configuration only.
 */
void
Mcp2515::setModeConfig(void)
{
    setMode(4);
}

/**
 * @brief Check if there are messages on any buffers
 * @pre Methods configRxBuf() and setFilterRx() have to be called first
 * @pre Device has to be in Normal mode
 * @post Buffer with valid message can be determined from getInterruptFlags()
 * @return Information about new messages in the buffers 0 and 1.\n
 * 0: No new messages\n
 * 1: New message in buffer 0\n
 * 2: New message in buffer 1\n
 * 3: New message in buffer 0 and 1
 */
uint8_t
Mcp2515::check4message(void)
{
    uint8_t regVal = 0;
    readReg(0x2C, &regVal, 1);

    uint8_t bufInfo = regVal & 0x03;

    return bufInfo;
}

/**
 * @brief Check if new received message is available
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @return 0: no new message, 1: new message
 * @pre Methods configRxBuf() and setFilterRx() have to be called first
 * @pre Device has to be in Normal mode
 * @post Buffer with valid message can be determined from getInterruptFlags()
 */
uint8_t
Mcp2515::check4message(uint8_t indRxb)
{
    uint8_t regVal = 0;
    uint8_t checkRes = 0;

    //#ifdef __unix
    //    // read RX status instruction - alternative on p. 65
    //    uint8_t dataIn[3] = {0x2C, 0};
    //    uint8_t dataOut[3] = {0, 0};

    //    _interface->readWriteBytes(dataOut, dataIn, 2);

    //    checkRes = (dataOut[1] >> indRxb) & 0x01;
    //#endif // __unix

    readReg(0x2C, &regVal, 1);

    checkRes = (regVal >> indRxb) & 0x01;

    return checkRes;
}

/**
 * @brief Wait until a message arrives. This method cannot be stopped.
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @pre Methods configRxBuf() and setFilterRx() have to be called first
 * @pre Device has to be in Normal mode
 */
void
Mcp2515::wait4message(uint8_t indRxb)
{
    while(!check4message(indRxb)) {
        // do nothing than waiting
    }
}

/**
 * @brief Determines the number of received bytes
 * @param[in] indRxb Index of the transmission buffer (0-1)
 * @return Number of received bytes
 */
uint8_t
Mcp2515::getNumBytesReceived(uint8_t indRxb)
{
    uint8_t ret = getDataLenCodeRx(indRxb);

    ret = ret & 0xF; // get the last 4 bytes of the register RXBnDLC

    return ret;
}

/**
 * @brief Disable rollover function from RXB0 to RXB1
 */
void
Mcp2515::disableRollover(void)
{
    modBit(0x60, 0b00000100, 0);
}

/**
 * @brief Enable rollover function from RXB0 to RXB1
 * @pre Device has to be in Configuration mode
 */
void
Mcp2515::enableRollover(void)
{
    modBit(0x60, 0b00000100, 0xFF);
}

/**
 * @brief Check if interrupts occured
 * @return Error code - ICOD2-0 (man. p. 57)
 */
uint8_t
Mcp2515::getStatus(void)
{
    uint8_t statReg = 0;
    readReg(0x0E, &statReg, 1);

    // determine error code
    uint8_t errCode = (statReg >> 1) && 0x07;

    return errCode;
}

/**
 * @brief Clears the interrupt error flags. This method has to be called if a message is received that is not of interest.
 */
void
Mcp2515::clearErrors(void)
{
    uint8_t val = 0;
    writeReg(0x2C, &val, 1);
    modBit(0x2D, 0b11000000, val);
}

/**
 * @brief Check if errors occured
 * @return Error flags (man. p. 47)
 * @note Returned value obtains the following errors:
 * - Bit 7: Receive Buffer 1 Overflow Flag
 * - Bit 6: Receive Buffer 0 Overflow Flag
 * - Bit 5: Bus-Off Error Flag
 * - Bit 4: Transmit Error-Passive Flag
 * - Bit 3: Receive Error-Passive Flag
 * - Bit 2: Transmit Error Warning Flag
 * - Bit 1: Receive Error Warning Flag
 * - Bit 0: Error Warning Flag
 */
uint8_t
Mcp2515::getErrorFlags(void)
{
    uint8_t errFlgs = 0;
    readReg(0x2D, &errFlgs, 1);

    return errFlgs;
}

/**
 * @brief Check reasons for interrupts
 * @return Error flags (man. p. 51)
 * @note Returned value obtains the following interrupt sources:
 * - Bit 7: Message Error Interrupt Flag
 * - Bit 6: Wakeup Interrupt Flag
 * - Bit 5: Error Interrupt Flag, refer to getErrorFlags()
 * - Bit 4: Transmit Buffer 2 Empty Interrupt Flag
 * - Bit 3: Transmit Buffer 1 Empty Interrupt Flag
 * - Bit 2: Transmit Buffer 0 Empty Interrupt Flag
 * - Bit 1: Receive Buffer 1 Full Interrupt Flag
 * - Bit 0: Receive Buffer 0 Full Interrupt Flag
 */
uint8_t
Mcp2515::getInterruptFlags(void)
{
    uint8_t intFlgs = 0;
    readReg(0x2C, &intFlgs, 1);

    return intFlgs;
}

/**
 * @brief Sets the value of the propagation segment
 * @param[in] ps Number of \f$TQ\f$ in propagation segment (1-8, manual p. 42)
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::setPropSeg(uint8_t ps)
{
    modBit(0x29, 0b00000111, ps - 1);
}

/**
 * @brief Sets the value of the time segment 1
 * @param[in] ps1 Number of \f$TQ\f$ in time segment 1 (1-8, manual p. 42)
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::setPs1(uint8_t ps1)
{
    modBit(0x29, 0b00111000, (ps1 - 1) << 3);
}

/**
 * @brief setPs2 Sets the value of the time segment 2
 * @param[in] ps2 Number of \f$TQ\f$ in time segment 2 (2-8, manual p. 42)
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::setPs2(uint8_t ps2)
{
    modBit(0x29, 0b10000000, 0xFF);
    modBit(0x28, 0b00000111, ps2 - 1);
}

/**
 * @brief Sets the length of the synchronization jump width
 * @param[in] sjw Number of \f$TQ\f$ in synchronization jump width (1-4, manual p. 42)
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::setSjw(uint8_t sjw)
{
    modBit(0x2A, 0b11000000, (sjw - 1) << 6);
}

/**
 * @brief Sets the length of one time quantum \f$TQ\f$. The needed prescaller is computed by
 *  @f[
 * brp = \frac{TQ \cdot f_{OSC}}{2}
 * @f]
 * where \f$TQ\f$ is the length of one time quantum and \f$f_{OSC}\f$ is the frequency of the used oscillator.
 * @param[in] brp Time Quantum code (baud rate prescaler - manual p. 42; 1 - 64)
 * @pre Device has to be in Configuration mode
 * @post setModeNormal() or similar has to be called afterwards.
 */
void
Mcp2515::setBrp(uint8_t brp)
{
    modBit(0x2A, 0b00111111, brp - 1);
}
