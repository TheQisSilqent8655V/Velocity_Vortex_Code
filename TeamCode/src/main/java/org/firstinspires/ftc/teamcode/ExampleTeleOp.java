package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Conno on 9/24/2016.
 */

@TeleOp(name = "Example TeleOp", group = "TeleOp")
public class ExampleTeleOp extends OpMode {

    double rightSpeed = 0.0;
    double leftSpeed = 0.0;
    double flywheelSpeed = 0.0;
    double targetFlywheelSpeed = 0.0;
    long timerVar = 0;

    DcMotor FRW; // Drive
    DcMotor BRW;
    DcMotor FLW;
    DcMotor BLW;

    DcMotor URFW; // Flywheel
    DcMotor LRFW;
    DcMotor ULFW;
    DcMotor LLFW;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        // Map Motors
        FLW = hardwareMap.dcMotor.get("FLW"); // Drive
        FRW = hardwareMap.dcMotor.get("FRW");
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");

        URFW = hardwareMap.dcMotor.get("URFW"); // Flywheel
        LRFW = hardwareMap.dcMotor.get("LRFW");
        ULFW = hardwareMap.dcMotor.get("ULFW");
        LLFW = hardwareMap.dcMotor.get("LLFW");

        // Reverse Motors
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
        FLW.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        if(gamepad1.right_bumper)
        {
            rightSpeed = gamepad1.right_stick_y;
            leftSpeed = gamepad1.right_stick_y;
        }
        else
        {
            rightSpeed = (gamepad1.right_stick_y / 2);
            leftSpeed = (gamepad1.right_stick_y / 2);
        }
        runRightWheels(rightSpeed);
        runLeftWheels(leftSpeed);
        if(targetFlywheelSpeed != flywheelSpeed && (timerVar == 0 || System.currentTimeMillis() > (timerVar + 40)))
        {
            flywheelSpeed += (((targetFlywheelSpeed - flywheelSpeed) / (Math.abs(targetFlywheelSpeed - flywheelSpeed))) * 0.01);
            timerVar = System.currentTimeMillis();
        }
        else
        {
            timerVar = 0;
        }
        telemetry.addData("Right Speed ", rightSpeed);
        telemetry.addData("Left Speed ", leftSpeed);
    }

    /*
     * runRightWheels - Function to run the right drive wheels
     * @param double speed - the speed
     */
    void runRightWheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        FRW.setPower(speed);
        BRW.setPower(speed);
    }

    /*
     * runLeftWheels - Function to run the left drive wheels
     * @param double speed - the speed
     */
    void runLeftWheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        FLW.setPower(speed);
        BLW.setPower(speed);
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
