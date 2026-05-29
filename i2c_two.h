#include "i2c.h"

/*******************************************************************************
    *
    * This function is to read data with 8-bit register address.
    *
    * @param   reg: I2C peripheral register base address.
    * @param   slaveAddr: Address of the slave device.
    * @param   regAddr: 8-bit register address.
    * @param   data: Pointer to the data buffer to store read data.
    * @param   length: Length of the data buffer.
    *
    * This function performs a read operation over I2C bus with an 8-bit register address.
    * It starts by sending the start sequence followed by the device address with the write bit.
    * Then, it sends the register address byte to access. After that, it sends the restart sequence
    * to switch from write mode to read mode. Next, it sends the device address byte with the read bit.
    * If the length of the data buffer is greater than one, it iterates through the buffer, sending
    * 0xFF to generate clock pulses while receiving data bytes from the slave. Finally, it sends the
    * stop sequence to complete the transaction.
    *
    *******************************************************************************/
        static void i2c_readData_b_2(u32 reg, u8 slaveAddr, u8 regAddr, u8 *data){
            i2c_masterStartBlocking(reg);               // Send start sequence
            i2c_txByte(reg, slaveAddr|I2C_WRITE);       // write device address byte with write bit
            i2c_txNackBlocking(reg);                    // send nack bit
            i2c_txByte(reg, (regAddr & 0xFF));          // write second byte address
            i2c_txNackBlocking(reg);                    // send nack bit
            i2c_masterRestartBlocking(reg);             // send restart sequence and wait for it to complete
            i2c_txByte(reg, slaveAddr|I2C_READ);        // write device address byte with read bit
            i2c_txNackBlocking(reg);                    // send nack bit

            i2c_txByte(reg, 0xFF);                      // send 0xFF (Release SDA line) to the slave while generate 8-bit SCL pulses
            i2c_txNackBlocking(reg);                    // send nack bit
            *data= i2c_rxData(reg);           			// read the data from rx data register and place it into last data array
            i2c_masterStopBlocking(reg);                // send stop sequence
        }



