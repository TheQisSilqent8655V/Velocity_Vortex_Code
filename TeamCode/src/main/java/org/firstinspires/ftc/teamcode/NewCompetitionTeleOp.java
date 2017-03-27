package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Conno on 3/10/2017.
 */
@TeleOp(name = "Test Vex TeleOp", group = "TeleOp")
public class NewCompetitionTeleOp extends NewDefineEverything {

    final double NORMAL_DRIVE_MULTIPLIER = 0.5;
    final double SLOW_DRIVE_MULTIPLIER = 0.25;
    double targetRightWheelSpeed = 0.0;
    double targetLeftWheelSpeed = 0.0;
    long dRTimerVar = 0;
    long dLTimerVar = 0;

    /**
     * Controls the speed of the flywheel based on encoder values. Also
     * controls the ramp down to make it fast and not kill the motors.
     * Uses just Bang Bang right now, might want to add PID for base
     * speed, but looking good right now.
     */
    Runnable BangBang = new Runnable() {
        public void run()
        {
            while (true)
            {
                // Measure flywheel speed with encoders
                /*do {
                    flywheelEncoderSpeed = Flywheel.getSpeed(0x10);
                }while (flywheelEncoderSpeed == -1);*/ // Throw out extraneous values
                /*error = Math.abs(flywheelEncoderSpeed - targetFlywheelEncoderSpeed);

                // Bang Bang if not ramping down, otherwise stop the flywheel as soon as you start intaking
                if(rampingDown)
                {
                    if(targetFlywheelSpeed == 0.0 && flywheelEncoderSpeed > 2)
                    {
                        if(gamepad1.left_bumper && flywheelEncoderSpeed < 30)
                        {
                            targetFlywheelSpeed = -0.2;
                            currentFlywheelSpeed = -0.2;
                        }
                    } else if ((targetFlywheelSpeed == -0.2 || targetFlywheelSpeed == 0.0) && flywheelEncoderSpeed < 3)
                    {
                        targetFlywheelSpeed = 0.0;
                        currentFlywheelSpeed = 0.0;
                        rampingDown = false;
                    }
                }
                else if((targetFlywheelEncoderSpeed == 70 && currentFlywheelSpeed > 0.45))  // If the flywheel is up to speed
                {
                    // If the flywheel is going slower than launching speed
                    if((error > 2 && flywheelEncoderSpeed < targetFlywheelEncoderSpeed))
                    {
                        // Bump up flywheel to different Bang Bang speeds depending on target speed
                        targetFlywheelSpeed = 0.82;
                        currentFlywheelSpeed = 0.82;
                    }
                    else
                    {
                        // If running okay run at normal speed
                        targetFlywheelSpeed = 0.54;
                        currentFlywheelSpeed = 0.54;
                    }
                }*/
                if(gamepad1.a)
                {
                    runIntake(1.0);
                    leftWheelSpeed = -0.3;
                    rightWheelSpeed = -0.3;
                    doDaSleep(300);
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(500);
                    runIntake(0.0);

                    while((stevens_IMU.getAngularOrientation().firstAngle * -1) < 20)
                    {
                        leftWheelSpeed = 0.3;
                        rightWheelSpeed = -0.3;
                    }

                    turnGyroWithCorrection(313);
                    leftWheelSpeed = -0.3;
                    rightWheelSpeed = -0.3;
                    doDaSleep(300);
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    targetFlywheelSpeed = 0.54;
                    doDaSleep(3000);
                    runIntake(1.0);
                    doDaSleep(5000);
                    runIntake(0.0);

                    rightWheelSpeed = -0.3;
                    leftWheelSpeed = -0.3;
                    doDaSleep(1000);
                    rightWheelSpeed = 0.0;
                    leftWheelSpeed = 0.0;
                    doDaSleep(500);
                }

                if(gamepad1.b)
                {
                    leftWheelSpeed = -0.3;
                    rightWheelSpeed = -0.3;
                    doDaSleep(300);
                    leftWheelSpeed = -0.4;
                    rightWheelSpeed = -0.4;
                    doDaSleep(450);
                    leftWheelSpeed = -0.3;
                    rightWheelSpeed = -0.3;
                    doDaSleep(400);
                    leftWheelSpeed = -0.2;
                    rightWheelSpeed = -0.2;
                    doDaSleep(400);
                    leftWheelSpeed = -0.1;
                    rightWheelSpeed = -0.1;
                    doDaSleep(400);
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(3000);
                    int turnError = (int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - 320);
                    double turnSpeed = 0.0;
                    while(((int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - 320)) > 7)
                    {
                        turnSpeed = 0.25;
                        leftWheelSpeed = turnSpeed;
                        rightWheelSpeed = -turnSpeed;
                    }
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(500);

                    // Press Beacons
                    while(Color.blue() < 3)
                    {
                        leftWheelSpeed = -0.25;
                        rightWheelSpeed = -0.25;
                    }
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(1000);
                    while(Color.blue() < 3)
                    {
                        leftWheelSpeed = 0.20;
                        rightWheelSpeed = 0.25;
                    }
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(600);

                    while(((int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - 312)) > 2)
                    {
                        turnSpeed = 0.25;
                        if(312 < (stevens_IMU.getAngularOrientation().firstAngle * -1))
                        {
                            turnSpeed = -turnSpeed;
                        }
                        leftWheelSpeed = -turnSpeed;
                        rightWheelSpeed = turnSpeed;
                     }
                    runBeaconPusher(0.5);

                    doDaSleep(100);
                    runBeaconPusher(0.0);
                    doDaSleep(200);
                    runBeaconPusher(-1.0);
                    doDaSleep(200);
                    runBeaconPusher(0.0);

                    while(((int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - 312)) > 5)
                    {
                        turnSpeed = 0.25;
                        if(312 < (stevens_IMU.getAngularOrientation().firstAngle * -1))
                        {
                            turnSpeed = -turnSpeed;
                        }
                        leftWheelSpeed = -turnSpeed;
                        rightWheelSpeed = turnSpeed;
                    }

                    leftWheelSpeed = -0.3;
                    rightWheelSpeed = -0.3;
                    doDaSleep(1000);

                    while(Color.blue() < 3)
                    {
                        leftWheelSpeed = -0.25;
                        rightWheelSpeed = -0.25;
                    }
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    doDaSleep(1000);
                    while(Color.blue() < 3)
                    {
                        leftWheelSpeed = 0.20;
                        rightWheelSpeed = 0.25;
                    }
                    doDaSleep(100);
                    leftWheelSpeed = 0.0;
                    rightWheelSpeed = 0.0;
                    runBeaconPusher(0.5);
                    doDaSleep(500);

                    runBeaconPusher(0.0);
                    doDaSleep(200);
                    runBeaconPusher(-1.0);
                    doDaSleep(200);
                    runBeaconPusher(0.0);

                    turnGyroWithCorrection(190);

                    rightWheelSpeed = -0.3;
                    leftWheelSpeed = -0.3;
                    doDaSleep(1000);
                    rightWheelSpeed = 0.0;
                    leftWheelSpeed = 0.0;
                    doDaSleep(500);

                    targetFlywheelSpeed = 0.54;
                    doDaSleep(5000);
                    runIntake(1.0);
                    doDaSleep(5000);
                    runIntake(0.0);
                    targetFlywheelSpeed = 0.0;

                    rightWheelSpeed = -0.3;
                    leftWheelSpeed = -0.3;
                    doDaSleep(1000);
                    rightWheelSpeed = 0.0;
                    leftWheelSpeed = 0.0;
                    doDaSleep(500);

                }
            }
        }
    };

    // Create Thread
    Thread VelocityControl = new Thread(BangBang);

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // Start the flywheel velocity control thread
        if(val == 0) {
            VelocityControl.start();
            runIntake(0.0);
            runBeaconPusher(0.0);
            val++;
        }

        //controlDrive(gamepad1.right_bumper, (gamepad1.right_trigger > 0.5)); // Turbo, Slow
        runLeftWheels(leftWheelSpeed);
        runRightWheels(rightWheelSpeed);

        controlFlywheel();

        //controlIntake(gamepad1.left_bumper, (gamepad1.left_trigger > 0.5)); // Intake, Outtake

        //controlBeaconPusher(gamepad1.dpad_left, gamepad1.dpad_right); // Inward, Outward

        controlLift(gamepad2.right_stick_y); // Lift power

        telemetry.addData("Target Right ", targetRightWheelSpeed);
        telemetry.addData("Right ", rightWheelSpeed);
        telemetry.addData("Power ", currentFlywheelSpeed);
        telemetry.addData("Gyro ", (stevens_IMU.getAngularOrientation().firstAngle * -1));

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
        if(turboButton)
        {
            targetRightWheelSpeed = gamepad1.right_stick_y;
            targetLeftWheelSpeed = gamepad1.left_stick_y;
        }
        else if(slowButton)
        {
            targetRightWheelSpeed = (gamepad1.right_stick_y * SLOW_DRIVE_MULTIPLIER);
            targetLeftWheelSpeed = (gamepad1.left_stick_y * SLOW_DRIVE_MULTIPLIER);
        }
        else
        {
            targetRightWheelSpeed = (gamepad1.right_stick_y * NORMAL_DRIVE_MULTIPLIER);
            targetLeftWheelSpeed = (gamepad1.left_stick_y * NORMAL_DRIVE_MULTIPLIER);
        }

        // Ramp the right side
        if(targetRightWheelSpeed != rightWheelSpeed /*&& (System.currentTimeMillis() > (dRTimerVar + 2))*/)
        {
            // If you are increasing in speed forward or backward
            if (Math.abs(targetRightWheelSpeed) > Math.abs(rightWheelSpeed))
            {
                if(Math.abs(Math.abs(targetRightWheelSpeed) - Math.abs(rightWheelSpeed)) >= 0.2)
                {
                    rightWheelSpeed = targetRightWheelSpeed;
                }
                else
                {
                    rightWheelSpeed = targetRightWheelSpeed;
                }
                //dRTimerVar = System.currentTimeMillis();
                rightWheelSpeed = (double)Math.round(rightWheelSpeed * 100d) / 100d;
            }
            else  // If you are decreasing in speed no need to ramp
            {
                rightWheelSpeed = targetRightWheelSpeed;
            }
        }
        else if(targetRightWheelSpeed == rightWheelSpeed)  // Reset to prepare for next ramp opportunity
        {
            //dRTimerVar = 0;
        }

        // Ramp the left side
        if(targetLeftWheelSpeed != leftWheelSpeed /*&& (System.currentTimeMillis() > (dLTimerVar + 2))*/)
        {
            // If you are increasing in speed forward or backward
            if (Math.abs(targetLeftWheelSpeed) > Math.abs(leftWheelSpeed))
            {
                if(Math.abs(Math.abs(targetLeftWheelSpeed) - Math.abs(leftWheelSpeed)) >= 0.2)
                {
                    rightWheelSpeed = targetRightWheelSpeed;
                }
                else
                {
                    leftWheelSpeed = targetLeftWheelSpeed;
                }
                //dLTimerVar = System.currentTimeMillis();
                leftWheelSpeed = (double)Math.round(leftWheelSpeed * 100d) / 100d;
            }
            else  // If you are decreasing in speed no need to ramp
            {
                leftWheelSpeed = targetLeftWheelSpeed;
            }
        }
        else if(targetLeftWheelSpeed == leftWheelSpeed)  // Reset to prepare for next ramp opportunity
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
        if(gamepad2.a)
        {
            targetFlywheelSpeed = 0.0;
            rampingDown = true;
        }
        else if(gamepad2.x)
        {
            targetFlywheelSpeed = 0.54;
            currentFlywheelSpeed = 0.54;
            targetFlywheelEncoderSpeed = 70;
        }
        else if(gamepad2.y)
        {

        }
        else if(gamepad2.b)
        {

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
        if(intake)
        {
            runIntake(1.0);
        }
        else if(outtake)
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
     * @param inward runs inward
     */
    void controlBeaconPusher(boolean outward, boolean inward)
    {
        if(outward) {
            runBeaconPusher(1.0);
        }
        else if(inward) {
            runBeaconPusher(-1.0);
        }
        else {
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

    void turnGyroWithCorrection(int targetPosition)
    {
        int turnError = (int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - targetPosition);
        double turnSpeed = 0.0;
        int stop = 60;
        if(turnError < 100)
        {
            stop = 15;
        }
        while(turnError > stop)
        {
            turnSpeed = 0.3;
            if(targetPosition < (stevens_IMU.getAngularOrientation().firstAngle * -1))
            {
                turnSpeed = -turnSpeed;
            }
            leftWheelSpeed = -turnSpeed;
            rightWheelSpeed = turnSpeed;
            turnError = (int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - targetPosition);
        }
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(1000);
        while(((int)Math.abs((stevens_IMU.getAngularOrientation().firstAngle * -1) - targetPosition)) > 7)
        {
            turnSpeed = 0.25;
            if(targetPosition < (stevens_IMU.getAngularOrientation().firstAngle * -1))
            {
                turnSpeed = -turnSpeed;
            }
            leftWheelSpeed = -turnSpeed;
            rightWheelSpeed = turnSpeed;
        }
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
    }




}
