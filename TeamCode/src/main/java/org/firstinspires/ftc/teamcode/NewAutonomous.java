package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Conno on 3/26/2017.
 */
@TeleOp(name = "New All Autonomous", group = "TeleOp")
public class NewAutonomous extends NewDefineEverything {

    // Class variables
    final int BEACON_AUTON = 0;      // Constants to make code readable
    final int JUST_LAUNCH_AUTON = 1;
    final int DATA_FORCE_AUTON = 2;
    final boolean BLUE = true;
    final boolean RED = false;

    double beaconPusherSpeed = 0.0;  // Variables for speeds not needed in teleop
    double intakeSpeed = 0.0;

    int currentAuton = DATA_FORCE_AUTON; // Variables for choosing autonomous
    boolean currentAlliance = RED;
    double currentDelay = 0.0;
    boolean currentParkStatus = true;
    boolean changingAutonType = false;
    boolean changingDelay = false;
    double speedMultiplier = 1.1;
    boolean correctingDrive = false;
    boolean correctingRedDrive = false;
    int correctingVal = 316;
    boolean bangBangControl = false;
    double adjustment = 1.0;
    boolean currentCap = false;
    double capAdjustment = 1.0;
    double capLauncherAdjustment = 1.0;
    double leftSpeedAdjustment = 1.0;


    Runnable correctDrive = new Runnable() {
        @Override
        public void run() {
            while(correctingDrive)
            {
                if(currentAlliance == RED)
                {
                    if (gyroReading() < (correctingVal - 1)) {
                        speedMultiplier = 0.7;
                    } else if (gyroReading() > (correctingVal + 1)) {
                        speedMultiplier = 1.3;
                    } else {
                        speedMultiplier = 1.1;
                    }
                }
                else {
                    if (gyroReading() > (correctingVal + 1)) {
                        speedMultiplier = 1.3;
                    } else if (gyroReading() < (correctingVal - 1)) {
                        speedMultiplier = 0.7;
                    } else {
                        speedMultiplier = 1.1;
                    }
                }
            }
            speedMultiplier = 1.1;
        }
    };

    Thread DoCorrection = new Thread(correctDrive);

    Runnable correctDriveRed = new Runnable() {
        @Override
        public void run() {
            while(correctingRedDrive)
            {
                if(gyroReading() > 200 && gyroReading() < 358)
                {
                    speedMultiplier = 0.7;
                }
                else if(gyroReading() > 2)
                {
                    speedMultiplier = 1.5;
                }
                else
                {
                    speedMultiplier = 1.1;
                }
            }
            speedMultiplier = 1.1;
        }
    };

    Thread DoCorrectionRed = new Thread(correctDriveRed);

    /**
     * Runs the autonomous in a thread so I don't
     * have to deal with state machines or conform
     * to the normie way of using linearOpMode
     */
    Runnable Autonomous = new Runnable() {
        public void run()
        {
            // Which autonomous to run, Alliance color, Delay(in seconds)
            allAutonomous(currentAuton, currentAlliance, currentDelay);
        }
    };

    // Create Thread
    Thread AllAutonomous = new Thread(Autonomous);

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void init_loop() {

        // Choose autonomous type
        if(gamepad1.y && !changingAutonType)
        {
            currentAuton++;
            if(currentAuton > DATA_FORCE_AUTON) // Loop if greater than last autonomous type
            {
                currentAuton = BEACON_AUTON;
            }
            changingAutonType = true;
        }
        else if(gamepad1.a && !changingAutonType)
        {
            currentAuton--;
            if(currentAuton < BEACON_AUTON) // Loop if less than first autonomous type
            {
                currentAuton = DATA_FORCE_AUTON;
            }
            changingAutonType = true;
        }
        else if(!gamepad1.y && !gamepad1.a)
        {
            changingAutonType = false; // Have this variable to not activate change multiple times when looping
        }

        // Choose alliance
        if(gamepad1.x)
        {
            currentAlliance = BLUE;
        }
        else if(gamepad1.b)
        {
            currentAlliance = RED;
        }

        // Choose delay
        if(gamepad1.dpad_up && !changingDelay)
        {
            currentDelay += 1;
            changingDelay = true;
        }
        else if(gamepad1.dpad_down && !changingDelay)
        {
            currentDelay -= 1;
            changingDelay = true;
        }
        else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
        {
            changingDelay = false; // Have this variable to not activate change multiple times when looping
        }



        // Display autonomous type
        if(currentAuton == BEACON_AUTON)
        {
            telemetry.addData("Auton Type ", "Beacon");
        }
        else if(currentAuton == JUST_LAUNCH_AUTON)
        {
            telemetry.addData("Auton Type ", "Just Launch");
        }
        else if(currentAuton == DATA_FORCE_AUTON)
        {
            telemetry.addData("Auton Type ", "Data Force");
        }

        // Display alliance color
        if(currentAlliance == BLUE)
        {
            telemetry.addData("Alliance ", "Blue");
        }
        else
        {
            telemetry.addData("Alliance ", "Red");
        }

        if(gamepad1.right_bumper)
        {
            currentCap = true;
            capAdjustment = 1.12;
            capLauncherAdjustment = 0.8;
        }
        else if(gamepad1.left_bumper)
        {
            currentCap = false;
            capAdjustment = 1.0;
            capLauncherAdjustment = 1.0;
        }

        if(gamepad1.right_trigger > 0.5)
        {
            currentParkStatus = true;
        }
        else if(gamepad1.left_trigger > 0.5)
        {
            currentParkStatus = false;
        }

        telemetry.addData("Cap ", currentCap);

        // Display delay
        telemetry.addData("Delay", currentDelay);
        telemetry.addData("Park? ", currentParkStatus);
        //telemetry.addData("Status ", stevens_IMU.getCalibrationStatus().toString());
        telemetry.addData("Status ", stevens_IMU.getSystemStatus().toString());
        telemetry.addData("Gyro ", gyroReading());
        telemetry.addData("Color ", colorReading(currentAlliance));

    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        // Start autonomous thread
        if(val == 0)
        {
            AllAutonomous.start();
            val++;
        }

        // Ramp the flywheel
        rampFlywheel();

        // Run all moving parts.
        // Don't use the run functions in a separate thread or the motors will
        // keep running even after stopping the program
        runRightWheels(((rightWheelSpeed * speedMultiplier) * capAdjustment));
        runLeftWheels(((leftWheelSpeed * leftSpeedAdjustment) * capAdjustment));
        runBeaconPusher(beaconPusherSpeed);
        runIntake(intakeSpeed);
        runFlywheel((currentFlywheelSpeed * capLauncherAdjustment));

        telemetry.addData("Gyro ", gyroReading());
        telemetry.addData("Color ", colorReading(currentAlliance));
        telemetry.addData("Right ", rightWheelSpeed);
        telemetry.addData("Left ",leftWheelSpeed);

    }

    @Override
    public void stop() {
        stopEverything();
    }

    /**
     * This is for convenience to stop all motors
     */
    void stopEverything()
    {
        targetFlywheelSpeed = 0.0; // Maybe want to do current as well
        rightWheelSpeed = 0.0;
        leftWheelSpeed = 0.0;
        beaconPusherSpeed = 0.0;
        intakeSpeed = 0.0;
        correctingDrive = false;
        correctingRedDrive = false;
        runVelocityControl = false;
    }

    /**
     * encoderDrives - Function to drive with encoder just at one speed
     *
     * @param speed      the power to run the motors at
     * @param ticksToGo  the amount of encoder ticks to go until stopping
     * @param direction  whether to go forward or backward
     */
    void encoderDrives(double speed, int ticksToGo, int direction)
    {

    }

    /**
     * This turns the robot at a certain speed until a certain gyro position
     * Use only positive speeds
     *
     * @param speed           the speed of the robot while turning
     * @param targetPosition  the position to turn to relative to starting
     */
    void gyroTurns(double speed, int targetPosition)
    {

    }

    /**
     * Used to make autonomous simpler to switch between red and blue autons
     *
     * @param color your alliance color
     * @return      value of specified color
     */
    int colorReading(boolean color)
    {
        if(color == RED)
        {
            return Color.red();
        }
        return Color.blue();
    }

    /**
     * Gets the reading of the gyro to make the autonomous code look more readable.
     * Multiplied by -1 because the gyro returns negatives
     *
     * @return the angle you are currently at
     */
    int gyroReading()
    {
        return (int)(stevens_IMU.getAngularOrientation().firstAngle * -1);
    }

    /**
     * Runs the drive at the desired powers. Made to make autonomous simpler.
     * Reversed the values because the drive direction is backward
     *
     * @param rightSpeed  power to set the right wheels to
     * @param leftSpeed   power to set the left wheels to
     */
    void setDriveSpeed(double rightSpeed, double leftSpeed)
    {
        rightWheelSpeed = -rightSpeed;
        leftWheelSpeed = -leftSpeed;
    }

    /**
     * This turns the robot with control to a target position
     *
     * @param targetPosition  the position to turn to relative to starting
     */
    void turnGyroWithCorrection(int targetPosition)
    {
        // Calculate how far away you are from target
        int turnError = Math.abs(gyroReading() - targetPosition);
        double turnSpeed;

        // Adjust for the increase in overshoot if farther away
        int stop = 60;
        if(turnError < 100)
        {
            stop = 15;
        }

        // Turn fast while far way from target
        while(turnError > stop)
        {
            turnSpeed = 0.3;

            // Change wheel directions based on which way you need to turn
            if(targetPosition < gyroReading())
            {
                turnSpeed = -turnSpeed;
            }
            setDriveSpeed(-turnSpeed, turnSpeed);

            // Re-calculate distance to target
            turnError = Math.abs(gyroReading() - targetPosition);
        }

        // Stop and let momentum stop
        setDriveSpeed(0.0, 0.0);
        doDaSleep(1000);

        // Calculate how close you are now
        turnError = Math.abs(gyroReading() - targetPosition);

        // Turn slower while close to target
        while(turnError > 7)
        {
            turnSpeed = (0.25 * adjustment);

            // Change wheel directions based on which way you need to turn
            if(targetPosition < gyroReading())
            {
                turnSpeed = -turnSpeed;
            }
            setDriveSpeed(-turnSpeed, turnSpeed);

            // Re-calculate distance to target
            turnError = Math.abs(gyroReading() - targetPosition);
        }

        // Stop when turn is completed
        setDriveSpeed(0.0, 0.0);
    }

    void betterGyroTurn(int targetPosition)
    {
        // Calculate how far away you are from target
        int turnError = Math.abs(gyroReading() - targetPosition);
        double turnSpeed;

        // Adjust for the increase in overshoot if farther away
        int stop = 15;
        if(turnError < 100)
        {
            stop = 7;
        }

        boolean stillTurning = true;
        boolean lessThan = false;

        turnSpeed = 0.3;

        // Change wheel directions based on which way you need to turn
        if(targetPosition < gyroReading())
        {
            turnSpeed = -turnSpeed;
            lessThan = true;
        }

        setDriveSpeed(-turnSpeed, turnSpeed);

        // Turn fast while far way from target
        while(stillTurning)
        {
            if(lessThan)
            {
                stillTurning = !(gyroReading() < (targetPosition + stop));
            }
            else
            {
                stillTurning = !(gyroReading() > (targetPosition - stop));
            }
        }

        turnSpeed = -turnSpeed;

        setDriveSpeed(-turnSpeed, turnSpeed);
        doDaSleep(300);

        // Stop when turn is completed
        setDriveSpeed(0.0, 0.0);
    }

    /**
     * Runs the autonomous you choose based on the parameters adjusted in the init loop
     * Constants to input are at the top of this file
     *
     * @param whichAuton type of autonomous to run
     * @param color      your alliance color
     * @param delay      the seconds you want to wait before running
     */
    void allAutonomous(int whichAuton, boolean color, double delay)
    {
        // Make sure nothing moves at the beginning and then delay
        long intakeTimer = System.currentTimeMillis();
        stopEverything();
        runVelocityControl = true;
        doDaSleep((int)(delay * 1000));

        // Choose autonomous type
        if(whichAuton == BEACON_AUTON)
        {
            double beaconAdjust = 1.0;
            if(color == BLUE)
            {
                beaconAdjust = -beaconAdjust;
            }
            DoCorrectionRed.start();
            correctingRedDrive = true;

            leftSpeedAdjustment = 1.1;

            if(color == RED)
            {
                // Drive to around first beacon while ramping the drive up and down
                setDriveSpeed(0.3, 0.3);
                doDaSleep(200);
                setDriveSpeed(0.4, 0.4);
                doDaSleep(200);
                setDriveSpeed(0.5, 0.5);
                doDaSleep(600);
                setDriveSpeed(0.4, 0.4);
                doDaSleep(200);
                setDriveSpeed(0.2, 0.2);
                doDaSleep(500);
                setDriveSpeed(0.1, 0.1);
                doDaSleep(500);
                setDriveSpeed(0.0, 0.0);
                doDaSleep(500);
                setDriveSpeed(-0.25, -0.25);
                doDaSleep(400);
                if(currentCap)
                {
                    doDaSleep(100);
                }
                setDriveSpeed(0.0, 0.0);
                doDaSleep(400);
            }
            else
            {
                // Drive to around first beacon while ramping the drive up and down
                setDriveSpeed(0.3, 0.3);
                doDaSleep(200);
                setDriveSpeed(0.4, 0.4);
                doDaSleep(200);
                setDriveSpeed(0.5, 0.5);
                doDaSleep(600);
                setDriveSpeed(0.4, 0.4);
                doDaSleep(200);
                setDriveSpeed(0.2, 0.2);
                doDaSleep(500);
                setDriveSpeed(0.1, 0.1);
                doDaSleep(500);
                setDriveSpeed(0.0, 0.0);
                doDaSleep(500);
                setDriveSpeed(-0.25, -0.25);
                doDaSleep(300);
                if(currentCap)
                {
                    doDaSleep(100);
                }
                setDriveSpeed(0.0, 0.0);
                doDaSleep(400);
            }

            correctingRedDrive = false;

            // Adjust turn based on alliance
            int firstTurnVal = 323;
            if(color == RED)
            {
                firstTurnVal = 42;
            }

            // Turn to be parallel with the wall
            double turnSpeed = 0.0;
            int margin = 7;
            if(color == RED)
            {
                margin = 3;
            }
            while(Math.abs(gyroReading() - firstTurnVal) > 3) // Want to finish within 7 degrees of 320
            {
                turnSpeed = 0.25; // Low speed to go slow and not overshoot
                if(color == RED)
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(turnSpeed, -turnSpeed);
            }
            turnSpeed = -turnSpeed;
            if(color == RED)
            {
                setDriveSpeed((turnSpeed - 0.12), -(turnSpeed - 0.12));
            }
            else
            {
                setDriveSpeed((turnSpeed - 0.08), -(turnSpeed - 0.08));
            }
            doDaSleep(200);
            setDriveSpeed(0.0, 0.0);

            correctingDrive = true;
            if(color == RED)
            {
                correctingVal = 46;
                leftSpeedAdjustment = 1.05;
            }
            DoCorrection.start();

            // Move until sensed desired color
            if(colorReading(color) < 3)
            {
                while (colorReading(color) < 3 || colorReading(color) > 100)
                {
                    setDriveSpeed(0.25, 0.25);
                }
                setDriveSpeed(-0.2, -0.2);
                doDaSleep(450);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);

            // Sense while going backwards in case of overshoot
            /*while(colorReading(color) < 3)
            {
                setDriveSpeed(-0.25, -0.25);
            }
            setDriveSpeed(0.2, 0.2);
            doDaSleep(150);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);*/

            // Adjust for your alliance
            int secondTurnVal = 317;
            if(color == RED)
            {
                secondTurnVal = 42;
            }

            // Correct to make sure you are parallel before you press
            /*while(Math.abs(gyroReading() - secondTurnVal) > 3)
            {
                turnSpeed = 0.25;
                if(secondTurnVal < gyroReading())
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(-turnSpeed, turnSpeed);
            }
            setDriveSpeed(0.0, 0.0);*/

            // Press beacon and retract beacon pusher
            beaconPusherSpeed = (0.5 * beaconAdjust);
            doDaSleep(750);
            beaconPusherSpeed = 0.0;
            doDaSleep(200);
            beaconPusherSpeed = (-1.0 * beaconAdjust);
            if(color == RED && currentCap)
            {
                doDaSleep(130);
            }
            else
            {
                doDaSleep(190);
            }
            if(color == BLUE)
            {
                doDaSleep(95);
            }
            beaconPusherSpeed = 0.0;

            // Re-adjust after press
            /*while(Math.abs(gyroReading() - secondTurnVal) > 6)
            {
                turnSpeed = 0.28;
                if(secondTurnVal < gyroReading())
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(-turnSpeed, turnSpeed);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(400);*/

            // Get passed first beacon before sensing again
            if(color == RED && currentCap)
            {
                correctingVal = 46;
            }
            setDriveSpeed(0.3, 0.3);
            doDaSleep(2000);

            // Sense the second beacon color
            while(colorReading(color) < 3 || colorReading(color) > 100)
            {
                setDriveSpeed(0.25, 0.25);
            }
            setDriveSpeed(-0.2, -0.2);
            doDaSleep(600);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);
            /*while(colorReading(color) < 3)
            {
                setDriveSpeed(-0.25, -0.25);
            }
            setDriveSpeed(0.2, 0.2);
            doDaSleep(150);
            setDriveSpeed(0.0, 0.0);*/

            // Press button
            beaconPusherSpeed = (0.5 * beaconAdjust);
            doDaSleep(750);
            beaconPusherSpeed = 0.0;
            doDaSleep(200);
            beaconPusherSpeed = (-1.0 * beaconAdjust);
            doDaSleep(130);
            beaconPusherSpeed = 0.0;

            correctingDrive = false;

            int targetPosition;

            // Turn to center vortex while adjusting a little bit for alliance
            if(color == RED)
            {
                targetPosition = 175;
                correctingVal = 175;
            }
            else
            {
                targetPosition = 190;
                correctingVal = 190;
            }

            // Start flywheel
            targetFlywheelSpeed = 0.52;
            targetFlywheelEncoderSpeed = 200;
            bangBangValue = 0.86;

            //betterGyroTurn(targetPosition);

            // Calculate how far away you are from target
            int turnError = Math.abs(gyroReading() - targetPosition);
            double turnSpeed2;

            // Adjust for the increase in overshoot if farther away
            int stop = 60;
            if(turnError < 100)
            {
                stop = 15;
            }

            // Turn fast while far way from target
            while(turnError > stop)
            {
                turnSpeed2 = 0.34;

                // Change wheel directions based on which way you need to turn
                if(targetPosition < gyroReading())
                {
                    turnSpeed2 = -turnSpeed2;
                }
                setDriveSpeed(-turnSpeed2, turnSpeed2);

                // Re-calculate distance to target
                turnError = Math.abs(gyroReading() - targetPosition);
            }

            // Start bang bang control
            bangBangControl = true;
            VelocityControl.start();

            // Stop and let momentum stop
            setDriveSpeed(0.0, 0.0);
            doDaSleep(1000);


            // Calculate how close you are now
            turnError = Math.abs(gyroReading() - targetPosition);

            // Turn slower while close to target
            /*while(turnError > 7)
            {
                turnSpeed2 = 0.25;
                if(color == RED)
                {
                    turnSpeed2 = 0.3;
                }

                // Change wheel directions based on which way you need to turn
                if(targetPosition < gyroReading())
                {
                    turnSpeed2 = -turnSpeed2;
                }
                setDriveSpeed(-turnSpeed2, turnSpeed2);

                // Re-calculate distance to target
                turnError = Math.abs(gyroReading() - targetPosition);
            }*/

            // Stop when turn is completed
            setDriveSpeed(0.0, 0.0);

            // Move toward center vortex
            if(color == RED)
            {
                long timeVar = System.currentTimeMillis();
                while(System.currentTimeMillis() < (timeVar + 1580))
                {
                    setDriveSpeed(0.3, 0.3);
                    if(gyroReading() > (184))
                    {
                        speedMultiplier = 1.3;
                    }
                    else if(gyroReading() < (180))
                    {
                        speedMultiplier = 0.9;
                    }
                    else
                    {
                        speedMultiplier = 1.1;
                    }
                }
            }
            else
            {
                long timeVar = System.currentTimeMillis();
                while(System.currentTimeMillis() < (timeVar + 1350))
                {
                    setDriveSpeed(0.3, 0.3);
                    if(gyroReading() > (187))
                    {
                        speedMultiplier = 1.3;
                    }
                    else if(gyroReading() < (183))
                    {
                        speedMultiplier = 0.9;
                    }
                    else
                    {
                        speedMultiplier = 1.1;
                    }
                }
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(2500);

            // Start flywheel and shoot
            intakeSpeed = 0.6;
            //doDaSleep((int)(28000 - (System.currentTimeMillis() - intakeTimer)));
            doDaSleep(5000);
            intakeSpeed = 0.0;
            targetFlywheelSpeed = 0.0;
            rampingDown = true;

            // Drive onto center platform
            setDriveSpeed(0.4, 0.4);
            doDaSleep(1000);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(500);
            setDriveSpeed(0.4, 0.4);
            doDaSleep(500);
        }
        else if(whichAuton == JUST_LAUNCH_AUTON)
        {
            // Start flywheel
            targetFlywheelSpeed = 0.52;
            bangBangValue = 0.82;
            flywheelEncoderSpeed = 200;

            long timeVar = System.currentTimeMillis();
            while(System.currentTimeMillis() < (timeVar + 1300))
            {
                setDriveSpeed(0.33, 0.33);
                if(gyroReading() > (300) && gyroReading() < (358))
                {
                    speedMultiplier = 0.9;
                }
                else if(gyroReading() < 100 && gyroReading() > 2)
                {
                    speedMultiplier = 1.3;
                }
                else
                {
                    speedMultiplier = 1.1;
                }
            }
            setDriveSpeed(0.0, 0.0);
            bangBangControl = true;
            VelocityControl.start();
            doDaSleep(5000);
            intakeSpeed = 0.7;
            int timeToWait = (int)(25000 - (System.currentTimeMillis() - intakeTimer));
            if(timeToWait > 0 && timeToWait < 30000)
            {
                doDaSleep(timeToWait);
            }
            else
            {
                doDaSleep(5000);
            }
            intakeSpeed = 0.0;
            targetFlywheelSpeed = 0.0;
            rampingDown = true;
            if(currentParkStatus)
            {
                setDriveSpeed(0.4, 0.4);
                doDaSleep(1300);
                setDriveSpeed(0.0, 0.0);
                doDaSleep(500);
            }
        }
        else if(whichAuton == DATA_FORCE_AUTON)
        {
            bangBangValue = 0.84;
            // Grab ball touching other robot
            intakeSpeed = 0.6;
            setDriveSpeed(0.3, 0.3);
            doDaSleep(700);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);
            setDriveSpeed(-0.3, -0.3);
            if(color == RED)
            {
                doDaSleep(200);
                intakeSpeed = 0.0;
                doDaSleep(200);
            }
            else
            {
                doDaSleep(300);
                intakeSpeed = 0.0;
                doDaSleep(450);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(2000);

            targetFlywheelSpeed = 0.50;
            bangBangValue2 = 0.50;
            targetFlywheelEncoderSpeed = 205;

            // Turn to center vortex
            if(color == RED)
            {
                setDriveSpeed(-0.4, 0.4);
                doDaSleep(400);
                turnGyroWithCorrection(49);
            }
            else
            {
                setDriveSpeed(0.4, -0.4);
                doDaSleep(350);
                turnGyroWithCorrection(313);
            }
            setDriveSpeed(0.0, 0.0);
            intakeSpeed = -1.0;
            doDaSleep(50);
            intakeSpeed = 0.0;
            doDaSleep(1300);

            // Move forward and shoot
            bangBangControl = true;
            VelocityControl.start();
            if(color == RED)
            {
                long timeVar = System.currentTimeMillis();
                while(System.currentTimeMillis() < (timeVar + 1250))
                {
                    setDriveSpeed(0.3, 0.3);
                    if(gyroReading() > (51))
                    {
                        speedMultiplier = 1.3;
                    }
                    else if(gyroReading() < (47))
                    {
                        speedMultiplier = 0.9;
                    }
                    else
                    {
                        speedMultiplier = 1.1;
                    }
                }
            }
            else
            {
                long timeVar = System.currentTimeMillis();
                while(System.currentTimeMillis() < (timeVar + 1200))
                {
                    setDriveSpeed(0.3, 0.3);
                    if(gyroReading() > (315))
                    {
                        speedMultiplier = 1.3;
                    }
                    else if(gyroReading() < (311))
                    {
                        speedMultiplier = 0.9;
                    }
                    else
                    {
                        speedMultiplier = 1.1;
                    }
                }
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(400);
            intakeSpeed = 0.6;
            int timeToWait = (int)(23000 - (System.currentTimeMillis() - intakeTimer));
            if(timeToWait > 0 && timeToWait < 30000)
            {
                doDaSleep(timeToWait);
            }
            else
            {
                doDaSleep(6000);
            }
            doDaSleep(timeToWait);
            intakeSpeed = 0.0;
            targetFlywheelSpeed = 0.0;
            rampingDown = true;
            setDriveSpeed(0.3, 0.3);
            doDaSleep(300);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);

            int daTurnVal = 220;
            // Drive onto corner vortex
            if(color == RED)
            {
                daTurnVal = 154;
            }
            adjustment = 1.1;
            turnGyroWithCorrection(daTurnVal);
            adjustment = 1.0;
            long timeVar = System.currentTimeMillis();
            while(System.currentTimeMillis() < (timeVar + 1500)) {
                setDriveSpeed(-0.4, -0.4);
                if(gyroReading() > (daTurnVal + 2))
                {
                    speedMultiplier = 0.7;
                }
                else if(gyroReading() < (daTurnVal - 2))
                {
                    speedMultiplier = 1.3;
                }
                else
                {
                    speedMultiplier = 1.1;
                }
            }
            setDriveSpeed(-0.2, -0.2);
            doDaSleep(2000);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(300);
        }

        stopEverything();
        doDaSleep(10000);
    }
}
