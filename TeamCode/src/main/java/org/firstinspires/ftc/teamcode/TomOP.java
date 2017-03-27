package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Conno on 3/10/2017.
 */
@TeleOp(name = "Tom Test", group = "TeleOp")
public class TomOP extends NewDefineEverything {
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        // Map Sensors
        FlywheelEncoder = hardwareMap.i2cDeviceSynch.get("F");
        //LeftDriveEncoder = hardwareMap.i2cDeviceSynch.get("LDE");

        Flywheel = new VEXEncoder(0x10, FlywheelEncoder);
        //LeftDrive = new VEXEncoder(0x11, LeftDriveEncoder);

    }
    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        //Just use 0x10 all the time
        telemetry.addData("RPosition: ", Flywheel.getPosition(0x10));
        //telemetry.addData("RSpeed: ", Flywheel.getVelocity(0x10));

        //telemetry.addData("LPosition: ", RightDrive.getPosition(0x11));
        //telemetry.addData("LSpeed: ", RightDrive.getVelocity(0x11));
    }
}
