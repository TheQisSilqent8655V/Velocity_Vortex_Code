package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Conno on 9/22/2016.
 */

@TeleOp(name = "Initialize and Get Heading", group = "Autonomous Tests")
public class Autonomous extends DefineEverything {

    // X and Y positions of robot
    double currentX = NOTHING;
    double currentY = NOTHING;

    // Speeds of the motors to be changed by autonomous thread
    double rightWheelSpeed = STOP_MOTOR_SPEED;
    double leftWheelSpeed = STOP_MOTOR_SPEED;

    // Variable for starting the autonomous thread
    int val = (int)NOTHING;

    /*
     * initializeAllSensorsSlashEncoders - Function to reset and initialize all da stuff
     */
    void initializeAllSensorsSlashEncoders() {
        // Initialize encoders
        rightEncoderMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoderMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        doDaSleep(ENCODER_CALIBRATION_WAIT_TIME);
        rightEncoderMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoderMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize color
        color.enableLed(true);

        // Calibrate gyro
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            doDaSleep(GYRO_CALIBRATION_WAIT_TIME);
        }
    }

    /*
     * encoderDrives - Function to drive with encoder just at one speed
     * @param double speed - the power to run the motors at
     * @param int ticksToGo - the amount of encoder ticks to go until stopping
     * @param boolean direction - whether to go forward or backward
     */
    void encoderDrives(double speed, int ticksToGo, int direction)
    {
        // Set up all variables
        int startingRightPosition = getRightWheelEncoderValue();
        int startingLeftPosition = getLeftWheelEncoderValue();
        double differenceInRightEncoders = Math.abs(getRightWheelEncoderValue() - startingRightPosition);
        double differenceInLeftEncoders;
        double differenceInWheels = NOTHING;

        // Drive to the specified distance
        while(differenceInRightEncoders < ticksToGo) // Using the right encoder to check distance
        {
            // Correct in wheels are off
            if(differenceInWheels < -DIFFERENCE_IN_WHEEL_ENCODER_THRESHOLD)
            {
                rightWheelSpeed = ((speed * ENCODER_DRIVE_STRAIGHT_CORRECTION) * direction);
                leftWheelSpeed = (speed * direction);
            }
            else if(differenceInWheels > DIFFERENCE_IN_WHEEL_ENCODER_THRESHOLD)
            {
                rightWheelSpeed = (speed * direction);
                leftWheelSpeed = ((speed * ENCODER_DRIVE_STRAIGHT_CORRECTION) * direction);
            }
            else
            {
                rightWheelSpeed = (speed * direction);
                leftWheelSpeed = (speed * direction);
            }

            // Update where encoder are and what the difference is
            differenceInRightEncoders = Math.abs(getRightWheelEncoderValue() - startingRightPosition);
            differenceInLeftEncoders = Math.abs(getLeftWheelEncoderValue() - startingLeftPosition);
            differenceInWheels = differenceInRightEncoders - differenceInLeftEncoders;
        }
    }

    /*
     * This turns the robot at a certain speed until a certain gyro position
     * Use only positive speeds
     * @param double speed - the speed of the robot while turning
     * @param int targetPosition - the position to turn to relative to starting
     */
    void gyroTurns(double speed, int targetPosition)
    {
        // Turn the correct direction
        if(targetPosition < gyro.getIntegratedZValue())
        {
            while(gyro.getIntegratedZValue() > targetPosition)
            {
                leftWheelSpeed = -speed;
                rightWheelSpeed = speed;
            }
        }
        else
        {
            while(gyro.getIntegratedZValue() < targetPosition)
            {
                leftWheelSpeed = speed;
                rightWheelSpeed = -speed;
            }
        }
    }

    /*
     * rampEncoderDrive - This function drives a certain distance using the encoders while
     *                    ramping up to not kill the motors and ramping down to come to a
     *                    stop at the desired target
     * @param int direction - the direction to drive, FORWARD or BACKWARD
     * @param int ticksToGo - the amount of encoder ticks to drive
     */
    void rampEncoderDrive(int direction, int ticksToGo)
    {
        // Set up all variables
        int starting = getRightWheelEncoderValue();
        int oneStepTotal = (RAMP_UP_ENCODER_STEP + RAMP_DOWN_ENCODER_STEP);
        int ticksToGoMaxSpeed = (oneStepTotal * RAMP_ENCODER_STEPS);
        double oneStepSpeedDifference = ((MAXIMUM_MOTOR_SPEED - RAMP_UP_START_SPEED) / RAMP_ENCODER_STEPS);

        // Ramp up and down based on how many encoder ticks to travel
        for(int checkDifference = (int)NOTHING; checkDifference <= (RAMP_ENCODER_STEPS); checkDifference++)
        {
            if(ticksToGo > (ticksToGoMaxSpeed - (oneStepTotal * checkDifference)))
            {
                for(int turnStep = (int)NOTHING; turnStep < (RAMP_ENCODER_STEPS - checkDifference); turnStep++)
                {
                    encoderDrives(((RAMP_UP_START_SPEED + (oneStepSpeedDifference * turnStep))), (RAMP_UP_ENCODER_STEP - SINGLE_STEP_OVERRUN_ADJUSTMENT), direction);
                }
                encoderDrives(((MAXIMUM_MOTOR_SPEED - (oneStepSpeedDifference * checkDifference))), ((ticksToGo - (ticksToGoMaxSpeed - (oneStepTotal * checkDifference))) - SINGLE_STEP_OVERRUN_ADJUSTMENT), direction);
                for(int turnStep = checkDifference; turnStep < RAMP_ENCODER_STEPS; turnStep++)
                {
                    encoderDrives((((MAXIMUM_MOTOR_SPEED - oneStepSpeedDifference) - (oneStepSpeedDifference * turnStep))), (RAMP_DOWN_ENCODER_STEP - SINGLE_STEP_OVERRUN_ADJUSTMENT), direction);
                }
                break;
            }
        }

        // Stop and correct if off
        leftWheelSpeed = STOP_MOTOR_SPEED;
        rightWheelSpeed = STOP_MOTOR_SPEED;
        doDaSleep(OVERRUN_CORRECTION_WAIT_TIME);
        int totalDistanceTraveled = Math.abs(getRightWheelEncoderValue() - starting);
        int distanceToCorrect = Math.abs(totalDistanceTraveled - ticksToGo);
        if(totalDistanceTraveled > ticksToGo)
        {
            encoderDrives(MINIMUM_WHEEL_POWER_TO_MOVE, distanceToCorrect, -direction); // Go back if to far
        }
        else
        {
            encoderDrives(MINIMUM_WHEEL_POWER_TO_MOVE, distanceToCorrect, direction); // Go forward if not far enough
        }

        // End with wheels stopped
        leftWheelSpeed = STOP_MOTOR_SPEED;
        rightWheelSpeed = STOP_MOTOR_SPEED;
    }

    /*
     * rampTurnGyro - This function turns a certain degree using the gyro while
     *                ramping up to not kill the motors and ramping down to come to a
     *                stop at the desired target
     * @param int targetPosition - The degree to turn to relative to the position of the
     *                             robot at the start of autonomous
     */
    void rampTurnGyro(int targetPosition)
    {
        // Set up all variables
        int difference = Math.abs(gyro.getIntegratedZValue() - targetPosition);
        int directionvalue = -GYRO_STEP_VALUE;
        if(gyro.getIntegratedZValue() < targetPosition)
            directionvalue = GYRO_STEP_VALUE;
        int oneStepTotal = (GYRO_STEP_VALUE + GYRO_STEP_VALUE);
        int degreesToGoMaxSpeed = (oneStepTotal * GYRO_RAMP_STEPS);
        double oneStepSpeedDifference = ((GYRO_TURN_MAX_SPEED - RAMP_UP_START_SPEED) / GYRO_RAMP_STEPS);
        int currentValueToTurnTo = gyro.getIntegratedZValue();
        int signOfDirection = (directionvalue / Math.abs(directionvalue));

        // Ramp up and down based on how many degrees to turn
        for(int checkdifference = (int)NOTHING; checkdifference <= (GYRO_RAMP_STEPS); checkdifference++)
        {
            if(difference > (degreesToGoMaxSpeed - (oneStepTotal * checkdifference)))
            {
                for(int turnstep = (int)NOTHING; turnstep < (GYRO_RAMP_STEPS - checkdifference); turnstep++)
                {
                    currentValueToTurnTo += (directionvalue - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                    gyroTurns(((RAMP_UP_START_SPEED + (oneStepSpeedDifference * turnstep))), currentValueToTurnTo);
                }
                currentValueToTurnTo += (((difference - (degreesToGoMaxSpeed - (oneStepTotal * checkdifference))) * signOfDirection) - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                gyroTurns((GYRO_TURN_MAX_SPEED - (oneStepSpeedDifference * checkdifference)), currentValueToTurnTo);
                for(int turnstep = checkdifference; turnstep < GYRO_RAMP_STEPS; turnstep++)
                {
                    currentValueToTurnTo += (directionvalue - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                    gyroTurns((GYRO_TURN_MAX_SPEED - (oneStepSpeedDifference * (turnstep++))), currentValueToTurnTo);
                }
                break;
            }
        }

        // This is the turning code that works but is kinda ugly
        // Still here and not in oldCode because the above code is not tested yet
        /*if(difference > 60)
        {
            gyroTurns(0.4, ((starting + directionvalue)) - 5);
            gyroTurns(0.6, ((starting + (directionvalue * 2)) - 10));
            gyroTurns(0.8, ((starting + ((difference - 60) * (directionvalue / Math.abs(directionvalue))) + (directionvalue * 2)) - 15));
            gyroTurns(0.6, (starting + (((((difference - 60) * (directionvalue / Math.abs(directionvalue)))) + (directionvalue * 3)) - 20)));
            gyroTurns(0.4, (starting + (((((difference - 60) * (directionvalue / Math.abs(directionvalue)))) + (directionvalue * 4)) - 25)));
        }
        else if(difference > 30)
        {
            gyroTurns(0.4, (starting + directionvalue));
            gyroTurns(0.6, (starting + directionvalue + ((difference - 30) * (directionvalue / Math.abs(directionvalue)))));
            gyroTurns(0.4, (starting + directionvalue + (directionvalue + ((difference - 30) * (directionvalue / Math.abs(directionvalue))))));
        }
        else
        {
            gyroTurns(0.4, (gyro.getIntegratedZValue() + ((difference) * (directionvalue / Math.abs(directionvalue)))));
        }*/

        // Stop and correct if off
        leftWheelSpeed = STOP_MOTOR_SPEED;
        rightWheelSpeed = STOP_MOTOR_SPEED;
        doDaSleep(OVERRUN_CORRECTION_WAIT_TIME);
        gyroTurns(MINIMUM_WHEEL_POWER_TO_MOVE, targetPosition);

        // End with wheels stopped
        leftWheelSpeed = STOP_MOTOR_SPEED;
        rightWheelSpeed = STOP_MOTOR_SPEED;
    }

    /*
     * turnRedBlue - Function to change turn based on alliance
     * @param double speed - the power to run the motors at
     * @param int targetPosition - the gyro position to turn to
     * @param boolean alliance - the alliance you are running from true for red, false for blue
     */
    void turnRedBlue(double speed, int targetPosition, boolean alliance)
    {

    }

    /*
     * driveTiles - This is to make it easier to call the encoder drive function
     * @param double tiles - The number of tiles to drive
     * @param int direction - The direction to move FORWARD or BACKWARD
     */
    void driveTiles(double tiles, int direction)
    {
        double encoderTicks = (tiles * ENCODER_TICKS_TO_MOVE_ONE_TILE);
        rampEncoderDrive(direction, (int)encoderTicks);
    }

    /*
     * isColorRed - Function to determine if color in front of sensor is blue or red
     * @return - True for red, false for blue
     */
    boolean isColorRed()
    {
        return (color.red() > color.blue());
    }

    /*
     * moveToPosition - This will move the robot to the desired position
     *                  It is based on cartesian coordinates with the origin being the
     *                  left corner from where you start and the middle of the robot
     *                  being the point
     *                  1 value for X or Y is 1 tile
     *                  MoveToPosition(1.5, .75) moves to 1.5 tiles up and .75 tiles to the
     *                  right of the origin
     * @param double x - The X coordinate to move to
     * @param double y - The Y coordinate to move to
     */
    void moveToPosition(double x, double y)
    {
        // Set up the triangle and solve for the hypotenuse and angle
        double dX = Math.abs(currentX - x);
        double dY = Math.abs(currentY - y);
        double driveDistance = (Math.sqrt((Math.pow(dX, POWER_OF_TWO)) + (Math.pow(dY, POWER_OF_TWO))));
        double angle = Math.toDegrees(Math.asin(((dX) / (driveDistance))));

        // Get starting values
        int actualAngle = gyro.getIntegratedZValue();
        int actualDistance = getRightWheelEncoderValue();

        // Adjust angle for what quadrant the triangle was in
        if(currentY > y)
        {
            angle = STRAIGHT_ANGLE - angle;
        }
        if(currentX < x)
        {
            angle = -angle;
        }

        // Turn then drive forward to new position and set it as current position
        rampTurnGyro((int)(angle + 0.5));
        actualAngle = Math.abs(gyro.getIntegratedZValue() - actualAngle); // Get the final turn value
        driveTiles(FORWARD, (int)(driveDistance + 0.5));
        actualDistance = Math.abs(actualDistance - getRightWheelEncoderValue()); // Get the final distance driven

        // Set the new position
        if(currentX > x)
            currentX -= (Math.cos(actualAngle) * actualDistance);
        else
            currentX -= (Math.cos(actualAngle) * actualDistance);

        if(currentY > y)
            currentY -= (Math.sin(actualAngle) * actualDistance);
        else
            currentY -= (Math.sin(actualAngle) * actualDistance);
    }

    /*
     * AutoPro - The thread to run the autonomous that sets the motor powers
     *           and lets the main loop set the motors to those speeds, doing
     *           this because this makes more sense to me than state machines
     */
    Runnable AutoPro = new Runnable() {
        public void run() {
            // Run autonomous
            moveToPosition(1, 1);

            // At the end of the autonomous routine stop motors
            rightWheelSpeed = STOP_MOTOR_SPEED;
            leftWheelSpeed = STOP_MOTOR_SPEED;
        }
    };

    // Create the autonomous thread
    Thread Autonomous = new Thread(AutoPro);

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {
        // Map Motors
        FLW = hardwareMap.dcMotor.get("FLW"); // Drive
        FRW = hardwareMap.dcMotor.get("FRW");
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");

        URFW = hardwareMap.dcMotor.get("URFW"); // Flywheel
        LRFW = hardwareMap.dcMotor.get("LRFW");
        ULFW = hardwareMap.dcMotor.get("ULFW");
        LLFW = hardwareMap.dcMotor.get("LLFW");

        // Map Sensors
        color = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse Motors
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
        FLW.setDirection(DcMotorSimple.Direction.REVERSE);

        initializeAllSensorsSlashEncoders();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // Start the autonomous thread only once
        if(val == (int)NOTHING)
        {
            Autonomous.start();
            val++;
        }

        // Run the motors at the speeds defined by the autonomous thread
        runLeftWheels(leftWheelSpeed);
        runRightWheels(rightWheelSpeed);
    }

}
