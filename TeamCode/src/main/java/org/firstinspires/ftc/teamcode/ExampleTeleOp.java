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
    double backIntakeSpeed = 0.5;
    double backFrontIntakeSpeed = 0.5;
    double frontFrontIntakeSpeed = 0.5;
    double beaconPusherSpeed = 0.5;
    boolean changeSpeed = false;

    DcMotor FRW; // Drive
    DcMotor BRW;
    DcMotor FLW;
    DcMotor BLW;

    DcMotor URFW; // Flywheel
    DcMotor LRFW;
    DcMotor ULFW;
    DcMotor LLFW;

    //Servo FI;
    Servo BI;
    Servo FFI;
    Servo BFI;
    Servo BP;



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

        //FI = hardwareMap.servo.get("FI");
        BI = hardwareMap.servo.get("BI");
        BFI = hardwareMap.servo.get("BFI");
        FFI = hardwareMap.servo.get("FFI");
        BP = hardwareMap.servo.get("BP");

        // Reverse Motors
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
        FRW.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse Flywheels
        ULFW.setDirection(DcMotorSimple.Direction.REVERSE);
        LRFW.setDirection(DcMotorSimple.Direction.REVERSE);

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
            leftSpeed = gamepad1.left_stick_y;
        }
        else
        {
            rightSpeed = (gamepad1.right_stick_y / 2);
            leftSpeed = (gamepad1.left_stick_y / 2);
        }
        runRightWheels(rightSpeed);
        runLeftWheels(leftSpeed);
        if(targetFlywheelSpeed != flywheelSpeed && (timerVar == 0 || System.currentTimeMillis() > (timerVar + 30)))
        {
            flywheelSpeed += (((targetFlywheelSpeed - flywheelSpeed) / (Math.abs(targetFlywheelSpeed - flywheelSpeed))) * 0.01);
            timerVar = System.currentTimeMillis();
        }
        else
        {
            timerVar = 0;
        }

        runFlywheels(flywheelSpeed);

        if(gamepad1.dpad_up)
        {
            backIntakeSpeed = 0.1;
        }
        else if(gamepad1.dpad_down)
        {
            backIntakeSpeed = 0.9;
        }
        else
        {
            backIntakeSpeed = 0.5;
        }

        if(gamepad1.left_bumper)
        {
            backFrontIntakeSpeed = 0.01;
            frontFrontIntakeSpeed = 0.01;
        }
        else if(gamepad1.left_trigger > 0)
        {
            backFrontIntakeSpeed = 0.99;
            frontFrontIntakeSpeed = 0.99;
        }
        else
        {
            backFrontIntakeSpeed = 0.5;
            frontFrontIntakeSpeed = 0.5;
        }

        if(gamepad1.dpad_left)
        {
            beaconPusherSpeed = 0.01;
        }
        else if(gamepad1.dpad_right)
        {
            beaconPusherSpeed = 0.99;
        }
        else
        {
            beaconPusherSpeed = 0.5;
        }

        BI.setPosition(backIntakeSpeed);
        BFI.setPosition(backFrontIntakeSpeed);
        FFI.setPosition(frontFrontIntakeSpeed);
        BP.setPosition(beaconPusherSpeed);

        if(gamepad1.a)
        {
            targetFlywheelSpeed = 0.0;
        }
        else if(gamepad1.b && (changeSpeed == false))
        {
            targetFlywheelSpeed += 0.01;
            changeSpeed = true;
        }
        else if(gamepad1.x && (changeSpeed == false))
        {
            targetFlywheelSpeed -= 0.01;
            changeSpeed = true;
        }
        else if(gamepad1.y)
        {
            targetFlywheelSpeed = 0.25;
        }
        else
        {
            changeSpeed = false;
        }
        telemetry.addData("Right Speed ", rightSpeed);
        telemetry.addData("Left Speed ", leftSpeed);
        telemetry.addData("Flywheel Speed", flywheelSpeed);
        telemetry.addData("Back Intake", backIntakeSpeed);
        telemetry.addData("Back Front Intake", backFrontIntakeSpeed);
        telemetry.addData("Front Front Intake", frontFrontIntakeSpeed);
        telemetry.addData("Beacon Pusher Speed", beaconPusherSpeed);
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

    void runFlywheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        ULFW.setPower(speed);
        URFW.setPower(speed);
        LLFW.setPower(speed);
        LRFW.setPower(speed);
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
