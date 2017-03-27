package org.firstinspires.ftc.teamcode;

/**
 * Created by Conno on 3/26/2017.
 */
public class NewAutonomous extends NewDefineEverything {

    // Class variables
    final int BEACON_AUTON = 0;      // Constants to make code readable
    final int JUST_LAUNCH_AUTON = 1;
    final int DATA_FORCE_AUTON = 2;
    final boolean BLUE = true;
    final boolean RED = false;

    double beaconPusherSpeed = 0.0;  // Variables for speeds not needed in teleop
    double intakeSpeed = 0.0;

    /**
     * Runs the autonomous in a thread so I don't
     * have to deal with state machines or conform
     * to the normie way of using linearOpMode
     */
    Runnable Autonomous = new Runnable() {
        public void run()
        {
            // Which autonomous to run, Alliance color, Delay(in seconds)
            allAutonomous(BEACON_AUTON, BLUE, 0);
        }
    };

    // Create Thread
    Thread AllAutonomous = new Thread(Autonomous);

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
        runRightWheels(rightWheelSpeed);
        runLeftWheels(leftWheelSpeed);
        runBeaconPusher(beaconPusherSpeed);
        runIntake(intakeSpeed);
        runFlywheel(currentFlywheelSpeed);


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
            turnSpeed = 0.25;

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

    /**
     * Runs the autonomous you choose based on the parameters adjusted in the init loop
     * Constants to input are at the top of this file
     *
     * @param whichAuton type of autonomous to run
     * @param color      your alliance color
     * @param delay      the seconds you want to wait before running
     */
    void allAutonomous(int whichAuton, boolean color, int delay)
    {
        // Make sure nothing moves at the beginning and then delay
        stopEverything();
        doDaSleep((delay * 1000));

        // Choose autonomous type
        if(whichAuton == BEACON_AUTON)
        {
            // Drive to around first beacon while ramping the drive up and down
            setDriveSpeed(0.3, 0.3);
            doDaSleep(300);
            setDriveSpeed(0.4, 0.4);
            doDaSleep(450);
            setDriveSpeed(0.3, 0.3);
            doDaSleep(400);
            setDriveSpeed(0.2, 0.2);
            doDaSleep(400);
            setDriveSpeed(0.1, 0.1);
            doDaSleep(400);

            // Stop for debugging, lower to 300 ms for real autonomous
            setDriveSpeed(0.0, 0.0);
            doDaSleep(3000);

            // Adjust turn based on alliance
            int firstTurnVal = 320;
            if(color == RED)
            {
                firstTurnVal = 40;
            }

            // Turn to be parallel with the wall
            double turnSpeed;
            while(Math.abs(gyroReading() - firstTurnVal) > 7) // Want to finish within 7 degrees of 320
            {
                turnSpeed = 0.25; // Low speed to go slow and not overshoot
                if(color == RED)
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(turnSpeed, -turnSpeed);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(500);

            // Move until sensed desired color
            while(colorReading(color) < 3)
            {
                setDriveSpeed(0.25, 0.25);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(1000);

            // Sense while going backwards in case of overshoot
            while(colorReading(color) < 3)
            {
                setDriveSpeed(-0.25, -0.2);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(600);

            // Adjust for your alliance
            int secondTurnVal = 312;
            if(color == RED)
            {
                secondTurnVal = 48;
            }

            // Correct to make sure you are parallel before you press
            while(Math.abs(gyroReading() - secondTurnVal) > 3)
            {
                turnSpeed = 0.25;
                if(secondTurnVal < gyroReading())
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(-turnSpeed, turnSpeed);
            }
            setDriveSpeed(0.0, 0.0);

            // Press beacon and retract beacon pusher
            runBeaconPusher(0.5);
            doDaSleep(400);
            runBeaconPusher(0.0);
            doDaSleep(200);
            runBeaconPusher(-1.0);
            doDaSleep(200);
            runBeaconPusher(0.0);

            // Re-adjust after press
            while(Math.abs(gyroReading() - secondTurnVal) > 5)
            {
                turnSpeed = 0.25;
                if(secondTurnVal < gyroReading())
                {
                    turnSpeed = -turnSpeed;
                }
                setDriveSpeed(-turnSpeed, turnSpeed);
            }
            setDriveSpeed(0.0, 0.0);

            // Get passed first beacon before sensing again
            setDriveSpeed(0.3, 0.3);
            doDaSleep(1000);

            // Sense the second beacon color
            while(colorReading(color) < 3)
            {
                setDriveSpeed(0.25, 0.25);
            }
            setDriveSpeed(0.0, 0.0);
            doDaSleep(1000);
            while(colorReading(color) < 3)
            {
                setDriveSpeed(-0.25, -0.2);
            }
            doDaSleep(100); // Wait a little to get past edge of color
            setDriveSpeed(0.0, 0.0);

            // Press button
            runBeaconPusher(0.5);
            doDaSleep(500);
            runBeaconPusher(0.0);
            doDaSleep(200);
            runBeaconPusher(-1.0);
            doDaSleep(200);
            runBeaconPusher(0.0);

            // Turn to center vortex while adjusting a little bit for alliance
            if(color == RED)
            {
                turnGyroWithCorrection(170);
            }
            else
            {
                turnGyroWithCorrection(190);
            }

            // Move toward center vortex
            setDriveSpeed(0.3, 0.3);
            doDaSleep(1000);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(500);

            // Start flywheel and shoot
            targetFlywheelSpeed = 0.54;
            doDaSleep(5000);
            runIntake(1.0);
            doDaSleep(5000);
            runIntake(0.0);
            targetFlywheelSpeed = 0.0;

            // Drive onto center platform
            /*setDriveSpeed(0.3, 0.3); // Commenting out because we might want to drive on corner vortex
            doDaSleep(1000);
            setDriveSpeed(0.0, 0.0);
            doDaSleep(500);*/
        }
        else if(whichAuton == JUST_LAUNCH_AUTON)
        {

        }
        else if(whichAuton == DATA_FORCE_AUTON)
        {

        }
        stopEverything();
        doDaSleep(5000);
    }



}
