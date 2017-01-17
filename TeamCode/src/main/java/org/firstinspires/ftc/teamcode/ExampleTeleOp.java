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
    boolean changeAccel = false;
    int val = 0;
    int flywheelEncoderSpeed = 0;
    int fillerval = 0;
    long flywheelTimerVar = 0;
    int rampTime = 30;
    double currentLaunchingSpeed = 0.22;
    boolean rampUp = false;
    int encoderThreshold = 45;
    double rampUpSpeed = 0.0;
    double rampUp2 = 0.30;
    boolean changeThresh = false;
    boolean changeRampSpeed = false;
    boolean launchingFirstRampUp = false;
    double oldLaunchSpeed = 0.0;
    long checkItCorrectly = 0;
    boolean rampDown = false;
    double speedAtWhichLaunching = currentLaunchingSpeed;

    double error = 0.0;
    double Kp = 0.0005;

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

    Runnable FlywheelSpeedCheck = new Runnable() {
        public void run() {
            while(true)
            {
                if(true)
                {
                    fillerval = URFW.getCurrentPosition();
                    doDaSleep(50);
                    flywheelEncoderSpeed = Math.abs((fillerval - URFW.getCurrentPosition()));
                    if (flywheelSpeed > (currentLaunchingSpeed - 0.01) && targetFlywheelSpeed > (currentLaunchingSpeed - 0.01) && oldLaunchSpeed == currentLaunchingSpeed
                            && !rampDown)
                    {
                        if(gamepad2.start && (backIntakeSpeed != 0.8 || frontFrontIntakeSpeed != 0.01))
                        {
                            error = flywheelEncoderSpeed - 100;
                            if(Math.abs(error) < 5)
                            {
                                error = 0.0;
                            }
                            currentLaunchingSpeed -= (error * Kp);
                            flywheelSpeed = currentLaunchingSpeed;
                            targetFlywheelSpeed = currentLaunchingSpeed;
                            rampUpSpeed = currentLaunchingSpeed + 0.3;
                        }
                        else
                        {
                            if (flywheelEncoderSpeed < 75)
                            {
                                flywheelSpeed = rampUpSpeed;
                                targetFlywheelSpeed = rampUpSpeed;
                            }
                            else
                            {
                                flywheelSpeed = currentLaunchingSpeed;
                                targetFlywheelSpeed = currentLaunchingSpeed;
                            }
                        }
                    }
                }
                if(rampDown)
                {
                    rampDown = !(flywheelSpeed < 0.05);
                }
                oldLaunchSpeed = currentLaunchingSpeed;
            }
        }
    };

    // Create the autonomous thread
    Thread RPM = new Thread(FlywheelSpeedCheck);



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
        LRFW = hardwareMap.dcMotor.get("IRD");
        ULFW = hardwareMap.dcMotor.get("ULFW");
        LLFW = hardwareMap.dcMotor.get("ILD");

        //FI = hardwareMap.servo.get("FI");
        BI = hardwareMap.servo.get("BI");
        BFI = hardwareMap.servo.get("BFI");
        FFI = hardwareMap.servo.get("FFI");
        BP = hardwareMap.servo.get("BP");

        // Reverse Motors
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
        FRW.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse Flywheels
        LLFW.setDirection(DcMotorSimple.Direction.REVERSE);

        ULFW.setDirection(DcMotorSimple.Direction.REVERSE);


        URFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        doDaSleep(100);
        URFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        if(val == 0)
        {
            RPM.start();
            val++;
            currentLaunchingSpeed = 0.3;
            rampUpSpeed = currentLaunchingSpeed + 0.3;
            rampUpSpeed = Range.clip(rampUpSpeed, 0.0, 1.0);
        }
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
        runLeftWheels((leftSpeed * 1.0));
        if(targetFlywheelSpeed != flywheelSpeed && (System.currentTimeMillis() > (timerVar + 60)))
        {
            flywheelSpeed += (((targetFlywheelSpeed - flywheelSpeed) / (Math.abs(targetFlywheelSpeed - flywheelSpeed))) * 0.01);
            timerVar = System.currentTimeMillis();
            flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
        }
        else if(targetFlywheelSpeed == flywheelSpeed)
        {
            timerVar = 0;
        }

        runFlywheels(flywheelSpeed);

        if(gamepad2.right_bumper && changeAccel == false)
        {
            backIntakeSpeed = 0.1;
            changeAccel = true;
        }
        else if(gamepad2.right_trigger > 0 && changeAccel == false)
        {
            backIntakeSpeed = 0.5;
            changeAccel = true;
        }
        else if(gamepad2.left_bumper && changeAccel == false)
        {
            backIntakeSpeed = 0.8;
            changeAccel = true;
        }
        else
        {
            changeAccel = false;
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

        rampUp = true;

        BI.setPosition(backIntakeSpeed);
        BFI.setPosition(backFrontIntakeSpeed);
        FFI.setPosition(frontFrontIntakeSpeed);
        BP.setPosition(beaconPusherSpeed);

        if(gamepad2.a)
        {
            targetFlywheelSpeed = 0.0;
            rampDown = true;
        }
        else if(gamepad2.b && (changeSpeed == false))
        {
            currentLaunchingSpeed += 0.01;
            targetFlywheelSpeed = currentLaunchingSpeed;
            changeSpeed = true;
        }
        else if(gamepad2.x && (changeSpeed == false))
        {
            currentLaunchingSpeed -= 0.01;
            targetFlywheelSpeed = currentLaunchingSpeed;
            changeSpeed = true;
        }
        else if(gamepad2.y)
        {
            targetFlywheelSpeed = currentLaunchingSpeed;
        }
        else
        {
            changeSpeed = false;
        }

        if(gamepad2.dpad_up && (changeRampSpeed == false))
        {
            rampUp2 += 0.01;
            changeRampSpeed = true;
        }
        else if(gamepad2.dpad_down && (changeRampSpeed == false))
        {
            rampUp2 -= 0.01;
            changeRampSpeed = true;
        }
        else
        {
            changeRampSpeed = false;
        }

        if(gamepad2.dpad_right && (changeThresh == false))
        {
            encoderThreshold += 1;
            changeThresh = true;
        }
        else if(gamepad2.dpad_left && (changeThresh == false))
        {
            encoderThreshold -= 1;
            changeThresh = true;
        }
        else
        {
            changeThresh = false;
        }

        /*if(checkItCorrectly == 0)
        {
            checkItCorrectly = System.currentTimeMillis();
        }
        else
        {
            if(System.currentTimeMillis() > checkItCorrectly + 5000 && flywheelSpeed != rampUpSpeed)
            {
                currentLaunchingSpeed = ((30 - this.hardwareMap.voltageSensor.iterator().next().getVoltage()) / encoderThreshold);
                rampUpSpeed = currentLaunchingSpeed + rampUp2;
                rampUpSpeed = Range.clip(rampUpSpeed, 0.0, 1.0);
                checkItCorrectly = 0;
            }
        }*/

        telemetry.addData("Right Speed ", rightSpeed);
        telemetry.addData("Left Speed ", leftSpeed);
        telemetry.addData("Flywheel Speed", flywheelSpeed);
        telemetry.addData("Back Intake", backIntakeSpeed);
        telemetry.addData("Encoder Threshold", encoderThreshold);
        telemetry.addData("Flywheel Encoder Speed", flywheelEncoderSpeed);
        telemetry.addData("Voltage ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Ramp Up Speed", rampUpSpeed);
        telemetry.addData("Current Launching Speed", currentLaunchingSpeed);
    }

    /*
     * runRightWheels - Function to run the right drive wheels
     * @param double speed - the speed
     */
    void runRightWheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        FRW.setPower(speed);
        BRW.setPower((Range.clip((speed * 1.0), -1.0, 1.0)));
        LRFW.setPower(speed);
    }

    /*
     * runLeftWheels - Function to run the left drive wheels
     * @param double speed - the power to run at
     */
    void runLeftWheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        FLW.setPower(speed);
        BLW.setPower((Range.clip((speed * 1.6), -1.0, 1.0)));
        LLFW.setPower(speed);
        //LRFW.setPower(speed);
    }

    void runFlywheels(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        ULFW.setPower(speed);
        URFW.setPower(speed);
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
