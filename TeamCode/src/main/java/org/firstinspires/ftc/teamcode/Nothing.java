package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * Created by Conno on 10/29/2016.
 * */
@TeleOp(name = "IMU loop test", group = "Autonomous Tests")
public class Nothing extends OpMode{


    AdafruitBNO055IMU stevens_IMU;
    BNO055IMU.Parameters parameters;
    boolean success;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        stevens_IMU = new AdafruitBNO055IMU(hardwareMap.i2cDeviceSynch.get("IMU"));

        parameters = new BNO055IMU.Parameters();

        success = stevens_IMU.initialize(parameters);
        telemetry.addData("Success: ", success);

        doDaSleep(500);

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        telemetry.addData("Success: ", success);
        telemetry.addData("Calibration Status", stevens_IMU.getCalibrationStatus());
        telemetry.addData("Gyro is Calibrated", stevens_IMU.isGyroCalibrated());
        telemetry.addData("Accelerometer is Calibrated", stevens_IMU.isAccelerometerCalibrated());
        telemetry.addData("Magnetometer is Calibrated", stevens_IMU.isMagnetometerCalibrated());
        telemetry.addData("System is Calibrated", stevens_IMU.isSystemCalibrated());
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Rotation: ", Round(stevens_IMU.getAngularOrientation().firstAngle * -57.143));
        telemetry.addData("Position ", stevens_IMU.getAngularOrientation());
    }

    private int Round(double num){
        return ((int)((num+0.5) * 10)/10);
    }

    /*
     * This makes the thread sleep for the entered amount of time
     * @param milli - This is the amount of time for the thread to sleep in milliseconds
     */
    void doDaSleep(int milli)
    {
        try {
            Thread.sleep(milli);
        } catch (InterruptedException ex) {

        }
    }

}
