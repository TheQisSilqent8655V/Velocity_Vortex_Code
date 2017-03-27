package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class VEXChain {
    //God and I were the only ones who understood this code
    //Now only god knows.

    /*
    Notes:
    When setting addresses, the I2cAddr can be constructed with either 7-bit or 8-bit address?
    When using the 0x4D address command, make sure to shift to 8-bit because of the write8 command.

     */

    //ADDRESSES
    private final I2cAddr DEFAULT = new I2cAddr(0x60);
    private final int ADDRESS_REGISTER = 0x4D;
    private final int POSITION_REGISTER = 0x40;
    private final int VELOCITY_REGISTER = 0x44;

    I2cDeviceSynch device;
    I2cAddr newAddress;

    public int currentSetAddress = 0;

    public VEXChain(int address, I2cDeviceSynch device){
        this.device = device;
        device.engage();

        I2cAddr defaultAddress = new I2cAddr(address);

        //Write and set new address
        device.write8(ADDRESS_REGISTER, address << 1);
        device.setI2cAddress(new I2cAddr(address));

        //Set no termination
        device.write8(0x4B, 1);

        //Reset to default
        device.setI2cAddress(defaultAddress);

        //Write and set second new address
        device.write8(ADDRESS_REGISTER, 0x12 << 1);
        device.setI2cAddress(new I2cAddr(0x11));

        //Set termination
        device.write8(0x4C, 1);
    }

    public boolean addressIsAvailable(int address){
        device.setI2cAddress(new I2cAddr(address));
        if(device.read8(0x20) == 0x00){
            return true;
        }
        return false;
    }

    /**
     * Gets the current position in ticks of the encoder.
     * @param encoder The 7-bit address of the encoder to read from
     * @return An unsigned integer of the position of the encoder.
     */
    public int getPosition(int encoder){
        //Set software I2c address
        device.setI2cAddress(new I2cAddr(encoder));

        //Read position bytes
        int position;
        byte[] vals = device.read(POSITION_REGISTER, 4);

        //Update current address variable for debugging
        currentSetAddress = device.getI2cAddress().get8Bit();

        //Bitwise OR and AND on the given bytes (bytes also have to be shifted)
        position = ((vals[0] << 8) & 0x0000ff00) | (vals[1] & 0x000000ff) | ((vals[3] << 16) & 0x00ff0000) | ((vals[2] << 24) & 0xff000000);
        return position;
    }

    /**
     * Gets the speed of the motors in ticks/50 ms
     * returns -1 if the values wrap around
     *
     * @param encoder the i2c address to read
     * @return distance the motors moves in 50 milliseconds
     */
    public int getSpeed(int encoder)
    {
        int fillerInt = getPosition(encoder);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ex) {

        }
        int fillerInt2 = getPosition(encoder);
        if(Math.abs(fillerInt - fillerInt2) < 500)
        {
            return Math.abs(fillerInt - fillerInt2);
        }
        return -1;
    }

    /**
     * Gets the unsigned velocity bytes from the velocity register.
     * @param encoder The address (in 7-bit) of the encoder to get the data from
     * @return An integer (unsigned) of the current velocity of the encoder.
     */
    public int getUnsignedVelocity(int encoder){
        device.setI2cAddress(new I2cAddr(encoder));
        int speed = 0;
        byte[] vals = device.read(VELOCITY_REGISTER, 2);

        speed = ((vals[0] << 8) & 0x0000ff00) | (vals[1] & 0x000000ff);
        return speed;
    }

    /**
     * Resets the device counter to 0 through the 0x4A register, sending a single bit
     * @param encoder The address (in 7-bit) of the encoder to reset.
     */
    public void zero(int encoder){
        device.setI2cAddress(new I2cAddr(encoder));
        device.write8(0x4A, 1);
    }

    /**
     * Gets the raw value that is returned from the first byte of the position register
     * @return The raw position value from the first position register byte
     */
    public int getRawAddressValue(){
        return device.read8(POSITION_REGISTER);
    }

    /**
     * Gets the 7-bit address of the selected device.
     * @return The address of the selected device in 7-bit format
     */
    public int getAddress(){
        return device.getI2cAddress().get7Bit();
    }
}
