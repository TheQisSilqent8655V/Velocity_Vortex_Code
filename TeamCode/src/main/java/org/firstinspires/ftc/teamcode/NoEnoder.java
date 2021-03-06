package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Conno on 3/25/2017.
 */
@TeleOp(name = "No Encoder", group = "TeleOp")
public class NoEnoder extends NewDefineEverything {

    // Class Variables
    final double NORMAL_DRIVE_MULTIPLIER = 0.5; // Constants to make code readable
    final double SLOW_DRIVE_MULTIPLIER = 0.25;

    double targetRightWheelSpeed = 0.0; // Drive ramp variables, not needed right now, but might come back to them
    double targetLeftWheelSpeed = 0.0;
    long dRTimerVar = 0;
    long dLTimerVar = 0;
    boolean changeBang = false;
    boolean changeBang2 = false;

    //TOM STUFF
    int POS = 0;
    int SSP = 0;
    int USP = 0;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        // Map Vex Motors
        FRW = hardwareMap.servo.get("FRW"); // Drive
        MRW = hardwareMap.servo.get("MRW");
        BRW = hardwareMap.servo.get("BRW");
        FLW = hardwareMap.servo.get("FLW");
        MLW = hardwareMap.servo.get("MLW");
        BLW = hardwareMap.servo.get("BLW");

        FRFW = hardwareMap.servo.get("FRFW"); // Flywheel
        BRFW = hardwareMap.servo.get("BRFW");
        FLFW = hardwareMap.servo.get("FLFW");
        BLFW = hardwareMap.servo.get("BLFW");

        Intake = hardwareMap.servo.get("Intake");
        BeaconPusher = hardwareMap.servo.get("BP");

        // Map Motors
        Lift = hardwareMap.dcMotor.get("L");

        // Reverse Vex Motors
        FLW.setDirection(Servo.Direction.REVERSE);
        MLW.setDirection(Servo.Direction.REVERSE);
        BLW.setDirection(Servo.Direction.REVERSE);
        FLFW.setDirection(Servo.Direction.REVERSE);
        BRFW.setDirection(Servo.Direction.REVERSE);

        // Reverse Motors
        Lift.setDirection(DcMotor.Direction.REVERSE);

        // Map Sensors
        FlywheelEncoder = hardwareMap.i2cDeviceSynch.get("F");
        Flywheel = new VEXEncoder(0x16, FlywheelEncoder);

        Color = hardwareMap.colorSensor.get("color");
        Color.enableLed(false);

        /*stevens_IMU = new AdafruitBNO055IMU(hardwareMap.i2cDeviceSynch.get("IMU")); // Adafruit Gyro
        parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        boolean success = stevens_IMU.initialize(parameters);*/

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // Start the flywheel velocity control thread
        if (val == 0) {
            VelocityControl.start();
            val++;
        }

        controlDrive(gamepad1.right_bumper, (gamepad1.right_trigger > 0.5)); // Turbo, Slow

        controlFlywheel();

        controlIntake(gamepad1.left_bumper, (gamepad1.left_trigger > 0.5)); // Intake, Outtake

        controlBeaconPusher(gamepad1.dpad_left, gamepad1.dpad_right); // Outward, Inward

        controlLift(gamepad2.right_stick_y); // Lift power

        if(gamepad2.back)
        {
            runVelocityControl = false;
        }

        if((gamepad1.y || gamepad2.dpad_up) && !changeBang)
        {
            bangBangValue2 += 0.01;
            changeBang = true;
        }
        else if((gamepad1.a || gamepad2.dpad_down) && !changeBang)
        {
            bangBangValue2 -= 0.01;
            changeBang = true;
        }
        else if(!gamepad1.y && !gamepad1.a && !gamepad2.dpad_up && !gamepad2.dpad_down)
        {
            changeBang = false;
        }

        if((gamepad1.x || gamepad2.dpad_left) && !changeBang2)
        {
            bangBangValue += 0.01;
            changeBang2 = true;
        }
        else if((gamepad1.b || gamepad2.dpad_right) && !changeBang2)
        {
            bangBangValue -= 0.01;
            changeBang2 = true;
        }
        else if(!gamepad1.x && !gamepad1.b && !gamepad2.dpad_left && !gamepad2.dpad_right)
        {
            changeBang2 = false;
        }

        // Debugging stuff
        telemetry.addData("Target Right ", targetRightWheelSpeed);
        telemetry.addData("Right ", rightWheelSpeed);
        telemetry.addData("Flywheel Encoder ", flywheelEncoderSpeed);
        telemetry.addData("Flywheel Speed ", flywheelEncoderSpeed);
        telemetry.addData("Bang Bang Up ", bangBangValue);
        telemetry.addData("Bang Bang ", bangBangValue2);
       //telemetry.addData("Blue ", Color.blue());
        //telemetry.addData("Gyro ", (stevens_IMU.getAngularOrientation().firstAngle * -1));

        //telemetry.addData("Position: ", POS);
        //telemetry.addData("Signed V: ", SSP);
        //telemetry.addData("Unsigned V: ", USP);


    }

    @Override
    public void stop() {
        runVelocityControl = false;
    }

    /**
     * Runs the drive based on the joysticks. Added the parameters
     * because I thought it would be easier to change and be read
     * easier in the loop.
     *
     * @param slowButton  if the slow button is pressed
     * @param turboButton if the turbo button is pressed
     */
    void controlDrive(boolean turboButton, boolean slowButton)
    {
        // Adjust speeds based on buttons
        if (turboButton)
        {
            targetRightWheelSpeed = gamepad1.right_stick_y;
            targetLeftWheelSpeed = gamepad1.left_stick_y;
        }
        else if (slowButton)
        {
            targetRightWheelSpeed = (gamepad1.right_stick_y * SLOW_DRIVE_MULTIPLIER);
            targetLeftWheelSpeed = (gamepad1.left_stick_y * SLOW_DRIVE_MULTIPLIER);
        }
        else
        {
            targetRightWheelSpeed = (gamepad1.right_stick_y * NORMAL_DRIVE_MULTIPLIER);
            targetLeftWheelSpeed = (gamepad1.left_stick_y * NORMAL_DRIVE_MULTIPLIER);
        }

        // Ramp the right side. Not using, but might come back to
        if (targetRightWheelSpeed != rightWheelSpeed /*&& (System.currentTimeMillis() > (dRTimerVar + 2))*/) {
            // If you are increasing in speed forward or backward
            if (Math.abs(targetRightWheelSpeed) > Math.abs(rightWheelSpeed)) {
                if (Math.abs(Math.abs(targetRightWheelSpeed) - Math.abs(rightWheelSpeed)) >= 0.2) {
                    //rightWheelSpeed += (((targetRightWheelSpeed - rightWheelSpeed) / (Math.abs(targetRightWheelSpeed - rightWheelSpeed))) * 0.2);
                    rightWheelSpeed = targetRightWheelSpeed;
                } else {
                    rightWheelSpeed = targetRightWheelSpeed;
                }
                //dRTimerVar = System.currentTimeMillis();
                rightWheelSpeed = (double) Math.round(rightWheelSpeed * 100d) / 100d;
            } else  // If you are decreasing in speed no need to ramp
            {
                rightWheelSpeed = targetRightWheelSpeed;
            }
        } else if (targetRightWheelSpeed == rightWheelSpeed)  // Reset to prepare for next ramp opportunity
        {
            //dRTimerVar = 0;
        }

        // Ramp the left side. Not using, but might come back to
        if (targetLeftWheelSpeed != leftWheelSpeed /*&& (System.currentTimeMillis() > (dLTimerVar + 2))*/) {
            // If you are increasing in speed forward or backward
            if (Math.abs(targetLeftWheelSpeed) > Math.abs(leftWheelSpeed)) {
                if (Math.abs(Math.abs(targetLeftWheelSpeed) - Math.abs(leftWheelSpeed)) >= 0.2) {
                    //leftWheelSpeed += (((targetLeftWheelSpeed - leftWheelSpeed) / (Math.abs(targetRightWheelSpeed - leftWheelSpeed))) * 0.2);
                    leftWheelSpeed = targetLeftWheelSpeed;
                } else {
                    leftWheelSpeed = targetLeftWheelSpeed;
                }
                //dLTimerVar = System.currentTimeMillis();
                leftWheelSpeed = (double) Math.round(leftWheelSpeed * 100d) / 100d;
            } else  // If you are decreasing in speed no need to ramp
            {
                leftWheelSpeed = targetLeftWheelSpeed;
            }
        } else if (targetLeftWheelSpeed == leftWheelSpeed)  // Reset to prepare for next ramp opportunity
        {
            //dLTimerVar = 0;
        }

        // Power the wheels
        runRightWheels(rightWheelSpeed);
        runLeftWheels(leftWheelSpeed);
    }

    /**
     * Sets the speed for the flywheel. Put the ramp speed in here
     * to not clutter the loop. Not going to put parameters like
     * controlDrive because the buttons are not constantly held
     * and it would look bad with that many parameters
     */
    void controlFlywheel()
    {
        // Set target speeds based on the buttons
        if (gamepad2.a)
        {
            targetFlywheelSpeed = 0.0;
            rampDownTimer = System.currentTimeMillis();
            rampingDown = true;
        }
        else if (gamepad2.x)
        {
            targetFlywheelSpeed = 1.0;
            currentFlywheelSpeed = 1.0;
            targetFlywheelEncoderSpeed = 205;
        }
        else if (gamepad2.y)
        {
            targetFlywheelSpeed = 1.0;
            currentFlywheelSpeed = 1.0;
            targetFlywheelEncoderSpeed = 190;
        }

        // Ramp the flywheel up and down to be nice to motors
        rampFlywheel();

        // Power the flywheel
        runFlywheel(currentFlywheelSpeed);
    }

    /**
     * Runs the intake based on what button is pressed. If
     * nothing is pressed the intake will not run.
     *
     * @param intake  run the intake inward
     * @param outtake run the intake outward
     */
    void controlIntake(boolean intake, boolean outtake)
    {
        if (intake)
        {
            runIntake(1.0);
        }
        else if (outtake)
        {
            runIntake(-1.0);
        }
        else
        {
            runIntake(0.0);
        }
    }

    /**
     * Runs the beacon pusher based on what button
     * is pressed. If nothing is pressed the beacon
     * pusher will not run.
     *
     * @param outward pushes out
     * @param inward  runs inward
     */
    void controlBeaconPusher(boolean outward, boolean inward)
    {
        if (outward)
        {
            runBeaconPusher(1.0);
        }
        else if (inward)
        {
            runBeaconPusher(-1.0);
        }
        else
        {
            runBeaconPusher(0.0);
        }
    }

    /**
     * Runs the lift
     *
     * @param gamepadInput the value of the gamepad stick to base lift on
     */
    void controlLift(double gamepadInput)
    {
        runLift(gamepadInput);
    }
}
