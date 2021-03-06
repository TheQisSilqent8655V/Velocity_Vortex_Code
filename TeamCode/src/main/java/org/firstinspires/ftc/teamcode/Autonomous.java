package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

/**
 * Created by Conno on 9/22/2016.
 */

@TeleOp(name = "All Autonomous", group = "Autonomous Tests")
public class Autonomous extends DefineEverything {

    // X and Y positions of robot
    double currentX = NOTHING;
    double currentY = NOTHING;

    // Speeds of the motors to be changed by autonomous thread
    double rightWheelSpeed = STOP_MOTOR_SPEED;
    double leftWheelSpeed = STOP_MOTOR_SPEED;
    double flywheelSpeed = STOP_MOTOR_SPEED;
    double targetFlywheelSpeed = STOP_MOTOR_SPEED;
    double frontIntakesSpeed = 0.5;
    double backIntakeSpeed = 0.5;
    double beaconPusherSpeed = 0.5;
    long timerVar = 0;
    int beaconPusherPosition = 1;

    // Autonomous variables to change in init loop
    boolean currentAlliance = BLUE;
    double[] currentStartingCoordinates = {0.0, 0.0};
    int currentDelay = 0;
    boolean[] currentBeaconsToGet = {false, false};
    boolean currentShootBeforeBeacons = false;
    boolean currentDefense = false;
    int currentDelayBeforeDefense = 0;
    ArrayList<double[]> currentMovementsForDefense = new ArrayList<>();
    int currentDelayBeforeShooting = 0;
    ArrayList<double[]> currentMovementsToShoot = new ArrayList<>();
    boolean currentEndOnCornerVortex = false;
    boolean currentSlowAutonomous = false;

    double leftWheelMultiplier = 1.18;

    // Variables for displaying information in init loop
    String[] headerStringArray = {"Red 2 Beacon Shoot Basic", "Blue 2 Beacon Shoot Basic", "Custom"}; // Variables that change with buttons
    int headerStringArrayLocation = 0;
    int sideToSideLocation = 0;
    int upAndDownLocation = 1;
    boolean changedHeader = false;

    String[] allParametersStringArray = new String[12]; // Final strings that display

    // Variable for starting the autonomous thread
    int val = (int)NOTHING;
    long timerVar2 = 0;
    boolean donePushing = false;
    double leftSpeedMultiplier = 1.6;
    double oldLaunchSpeed = 0.0;
    boolean rampDown = false;

    int daAutoDelay = 0;
    boolean daBlueAuto = false;
    boolean launchForwardAuto = false;
    boolean pressedDaChange = false;
    boolean defense = false;

    /*
     * initializeAllSensorsSlashEncoders - Function to reset and initialize all da stuff
     */
    void initializeAllSensorsSlashEncoders() {
        // Initialize encoders
        rightEncoderMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoderMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        URFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ULFW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        doDaSleep(ENCODER_CALIBRATION_WAIT_TIME);
        rightEncoderMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoderMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        URFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ULFW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize color
        color.enableLed(false);

        // Calibrate gyro
        gyro.calibrate();

        while (gyro.isCalibrating()) {
            doDaSleep(GYRO_CALIBRATION_WAIT_TIME);
        }
    }


    int flywheelEncoderSpeed = 0;
    int fillerval = 0;
    boolean keepRunning = true;

    Runnable FlywheelSpeedCheck = new Runnable() {
        public void run()
        {
            while(keepRunning)
            {
                fillerval = leftEncoderMotor().getCurrentPosition();
                doDaSleep(50);
                flywheelEncoderSpeed = Math.abs((fillerval - leftEncoderMotor().getCurrentPosition()));
                if (flywheelSpeed > (0.27 - 0.01) && targetFlywheelSpeed > (0.27 - 0.01))
                {
                        if (flywheelEncoderSpeed < 80)
                        {
                            flywheelSpeed = 0.53;
                            targetFlywheelSpeed = 0.53;
                        }
                        else
                        {
                            flywheelSpeed = 0.27;
                            targetFlywheelSpeed = 0.27;
                        }
                }
            }
            flywheelSpeed = 0.27;
            targetFlywheelSpeed = 0.27;
        }
    };

    // Create the autonomous thread
    Thread RPM = new Thread(FlywheelSpeedCheck);

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
                rightWheelSpeed = (speed * direction);
                leftWheelSpeed = ((speed * ENCODER_DRIVE_STRAIGHT_CORRECTION) * direction);
            }
            else if(differenceInWheels > DIFFERENCE_IN_WHEEL_ENCODER_THRESHOLD)
            {
                rightWheelSpeed = ((speed * ENCODER_DRIVE_STRAIGHT_CORRECTION) * direction);
                leftWheelSpeed = (speed * direction);
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
        if(targetPosition > gyro.getIntegratedZValue())
    {
        while(targetPosition > gyro.getIntegratedZValue())
        {
            leftWheelSpeed = speed;
            rightWheelSpeed = -speed;
        }
    }
    else
    {
        while(targetPosition < gyro.getIntegratedZValue())
        {
            leftWheelSpeed = -speed;
            rightWheelSpeed = speed;
        }
    }
    }

    /*
    * This turns the robot at a certain speed until a certain gyro position
    * Use only positive speeds
    * @param double speed - the speed of the robot while turning
    * @param int targetPosition - the position to turn to relative to starting
    */
    void gyroTurnsDifferentWheels(double speedL, double speedR, int targetPosition)
    {
        // Turn the correct direction
        if(targetPosition > gyro.getIntegratedZValue())
        {
            while(targetPosition > gyro.getIntegratedZValue())
            {
                leftWheelSpeed = speedL;
                rightWheelSpeed = speedR;
            }
        }
        else
        {
            while(targetPosition < gyro.getIntegratedZValue())
            {
                leftWheelSpeed = speedL;
                rightWheelSpeed = speedR;
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
                    encoderDrives(((((MAXIMUM_MOTOR_SPEED - 0.1) - oneStepSpeedDifference) - (oneStepSpeedDifference * turnStep))), (RAMP_DOWN_ENCODER_STEP - SINGLE_STEP_OVERRUN_ADJUSTMENT), direction);
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
        int directionValue = -GYRO_STEP_VALUE;
        if(gyro.getIntegratedZValue() < targetPosition)
            directionValue = GYRO_STEP_VALUE;
        int oneStepTotal = (GYRO_STEP_VALUE + GYRO_STEP_VALUE);
        int degreesToGoMaxSpeed = (oneStepTotal * GYRO_RAMP_STEPS);
        double oneStepSpeedDifference = ((GYRO_TURN_MAX_SPEED - RAMP_UP_START_SPEED) / GYRO_RAMP_STEPS);
        int currentValueToTurnTo = gyro.getIntegratedZValue();
        int signOfDirection = (directionValue / Math.abs(directionValue));

        // Ramp up and down based on how many degrees to turn
        for(int checkDifference = (int)NOTHING; checkDifference <= (GYRO_RAMP_STEPS); checkDifference++)
        {
            if(difference > (degreesToGoMaxSpeed - (oneStepTotal * checkDifference)))
            {
                for(int turnStep = (int)NOTHING; turnStep < (GYRO_RAMP_STEPS - checkDifference); turnStep++)
                {
                    currentValueToTurnTo += (directionValue - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                    gyroTurns(((RAMP_UP_START_SPEED + (oneStepSpeedDifference * turnStep))), currentValueToTurnTo);
                }
                currentValueToTurnTo += (((difference - (degreesToGoMaxSpeed - (oneStepTotal * checkDifference))) * signOfDirection) - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                gyroTurns((GYRO_TURN_MAX_SPEED - (oneStepSpeedDifference * checkDifference)), currentValueToTurnTo);
                for(int turnStep = checkDifference; turnStep < GYRO_RAMP_STEPS; turnStep++)
                {
                    currentValueToTurnTo += (directionValue - (SINGLE_STEP_OVERTURN_ADJUSTMENT * signOfDirection));
                    gyroTurns((GYRO_TURN_MAX_SPEED - (oneStepSpeedDifference * (turnStep++))), currentValueToTurnTo);
                }
                break;
            }
        }

        // This is the turning code that works but is kinda ugly and not easy to modify
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
     * putBeaconPusherInPosition - Puts the beacon pusher in the desired position
     * @param int position - The position to run to, 1-3
     */
    void putBeaconPusherInPosition(int position)
    {
        if(position != beaconPusherPosition)
        {
            if(beaconPusherPosition == 1)
            {
                beaconPusherSpeed = 0.01;
                doDaSleep(150);
                if(position == 3)
                {
                    doDaSleep(700);
                }
                beaconPusherSpeed = 0.5;
            }
            else if(beaconPusherPosition == 2)
            {
                if(position == 1)
                {
                    beaconPusherSpeed = 0.99;
                    doDaSleep(150);
                    beaconPusherSpeed = 0.5;
                }
                else
                {
                    beaconPusherSpeed = 0.01;
                    doDaSleep(600);
                    beaconPusherSpeed = 0.5;
                }
            }
            else
            {
                beaconPusherSpeed = 0.99;
                doDaSleep(700);
                if(position == 1)
                {
                    doDaSleep(150);
                }
                beaconPusherSpeed = 0.5;
            }
        }
        beaconPusherPosition = position;
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
     * angleToTurnToLocation - Figures out the angle to turn to look
     *                         at a different location
     * @param double x - The x coordinate of the different location
     * @param double y - The y coordinate of the different location
     * @return int - The angle to turn to
     */
    int angleToTurnToLocation(double x, double y)
    {
        double dX = Math.abs(currentX - x);
        double dY = Math.abs(currentY - y);
        double hypotenuse = (Math.sqrt((Math.pow(dX, 2)) + (Math.pow(dY, 2))));
        double angle = Math.toDegrees(Math.asin(((dX) / (hypotenuse))));
        if(currentY > y)
        {
            angle = 180 - angle;
        }
        if(currentX < x)
        {
            angle = -angle;
        }
        return ((int)(angle + 0.5));
    }

    int angle = 0;
    double driveDistance = 0.0;

    /*
     * moveToPosition - This will move the robot to the desired position
     *                  It is based on cartesian coordinates with the origin being the
     *                  left corner from where you start and the middle of the robot
     *                  being the point
     *                  1 value for X or Y is 1 tile
     *                  moveToPosition(1.5, .75) moves to 1.5 tiles up and .75 tiles to the
     *                  right of the origin
     * @param double x - The X coordinate to move to
     * @param double y - The Y coordinate to move to
     */
    void moveToPosition(double x, double y)
    {
        // Set up the triangle and solve for the hypotenuse and angle
        double dX = Math.abs(currentX - x);
        double dY = Math.abs(currentY - y);
        driveDistance = (Math.sqrt((Math.pow(dX, POWER_OF_TWO)) + (Math.pow(dY, POWER_OF_TWO))));
        angle = angleToTurnToLocation(x, y);

        // Get starting values
        int actualAngle = gyro.getIntegratedZValue();
        double actualDistance = getRightWheelEncoderValue();
        int direction = FORWARD;

        if(Math.abs(Math.abs(angle) - Math.abs(gyro.getIntegratedZValue())) > 90)
        {
            if(angle > 0)
            {
                angle -= STRAIGHT_ANGLE;
            }
            else
            {
                angle += STRAIGHT_ANGLE;
            }
            direction = BACKWARD;
        }

        // Turn then drive forward to new position and set it as current position
        if(currentSlowAutonomous)
            gyroTurns(MINIMUM_WHEEL_POWER_TO_MOVE, angle);
        else
            rampTurnGyro(angle);
        actualAngle = Math.abs(gyro.getIntegratedZValue()); // Get the final turn value
        if(currentSlowAutonomous)
            encoderDrives(MINIMUM_WHEEL_POWER_TO_MOVE, (int)(driveDistance * ENCODER_TICKS_TO_MOVE_ONE_TILE), direction);
        else
            driveTiles(direction, (int) (driveDistance + 0.5));
        actualDistance = (Math.abs(actualDistance - getRightWheelEncoderValue()) / ENCODER_TICKS_TO_MOVE_ONE_TILE); // Get the final distance driven

        // Set the new position
        if(currentX > x)
            currentX -= Math.abs((Math.cos(actualAngle) * actualDistance));
        else
            currentX += Math.abs((Math.cos(actualAngle) * actualDistance));

        if(currentY > y)
            currentY -= Math.abs((Math.sin(actualAngle) * actualDistance));
        else
            currentY += Math.abs((Math.sin(actualAngle) * actualDistance));
    }

    /*
     * positionRedBlue - To run in order to mirror autonomous based on alliance
     * @param double x - The x position to move to
     * @param double y - The y position to move to
     * @param alliance - The alliance that you are on
     */
    void positionRedBlue(double x, double y, boolean alliance)
    {
        if(alliance == RED)
        {
            moveToPosition((6 - x), y);
        }
        else
        {
            moveToPosition(x, y);
        }
    }

    /*
     * allAutonomous - The function to run for autonomous. Can be customized for any situation
     * @param boolean alliance - The alliance you are currently on
     * @param double[] startingCoordinates - The coordinated the middle of the robot starts at
     * @param int delay - The milliseconds to wait before starting autonomous
     * @param boolean[] beaconsToGet - Which beacons you want to push
     * @param boolean shootBeforeBeacon - If you want to shoot into the center goal before going
     *                                    for the beacons
     * @param boolean defense - If you want to play defense on the opponents beacons
     * @param int delayBeforeDefense - The amount of milliseconds to wait before defense
     * @param double[][] movementsForDefense - The locations to move to play defense
     * @param int delayBeforeShooting - The delay to wait before going to shoot.
     *                                  0 if no delay or shooting before beacons
     *                                  Over 30000 if not shooting
     * @param double[][] movementsToShoot - The locations to run to in order to shoot in center vortex
     * @param boolean endOnCornerVortex - If you want to end on the corner vortex ramp to catch the
     *                                    extra balls that fall down the ramp
     * @param boolean slowAutonomous - Run the autonomous slower to be more accurate and maybe bait
     *                                 teams into trying to block us
     */
    void allAutonomous(boolean alliance, double[] startingCoordinates, int delay,
                       boolean[] beaconsToGet, boolean shootBeforeBeacons, boolean defense,
                       int delayBeforeDefense, ArrayList<double[]> movementsForDefense, int delayBeforeShooting,
                       ArrayList<double[]> movementsToShoot, boolean endOnCornerVortex, boolean slowAutonomous)
    {
        if(slowAutonomous)
        {
            OVERRUN_CORRECTION_WAIT_TIME = 400;
            RAMP_DOWN_ENCODER_STEP = 600;
            RAMP_UP_ENCODER_STEP = 400;
            GYRO_STEP_VALUE = 50;
        }
        currentX = startingCoordinates[0];
        currentY = startingCoordinates[1];
        doDaSleep(delay);
        moveToPosition(currentX, 0.5);

        if(shootBeforeBeacons)
        {
            for(int i = 0; i < movementsToShoot.size(); i++)
            {
                moveToPosition(movementsToShoot.get(i)[0], movementsToShoot.get(i)[1]);
            }

            if(alliance == RED)
            {
                rampTurnGyro(angleToTurnToLocation(2.5, 2.5));
            }
            else
            {
                rampTurnGyro(angleToTurnToLocation(3.5, 2.5));
            }

            // Run flywheel and shoot
            targetFlywheelSpeed = 0.25;
            while(targetFlywheelSpeed != flywheelSpeed)
            {
                doDaSleep(10);
            }
            frontIntakesSpeed = 0.01;
            backIntakeSpeed = 0.99;
            doDaSleep(5000);
            frontIntakesSpeed = 0.5;
            backIntakeSpeed = 0.5;
            targetFlywheelSpeed = 0.0;
            while(targetFlywheelSpeed != flywheelSpeed)
            {
                doDaSleep(10);
            }
        }

        if(beaconsToGet[0])
        {
            //0.02083333
            if(alliance == RED)
            {
                positionRedBlue(5.5, 2.62, alliance);
                rampTurnGyro(180);
            }
            else
            {
                positionRedBlue(5.5, 2.12, alliance);
                rampTurnGyro(0);
            }
            putBeaconPusherInPosition(2);
            if(isColorRed() == alliance)
            {
                putBeaconPusherInPosition(3);
                doDaSleep(200);
                putBeaconPusherInPosition(2);
            }
            else
            {
                if(alliance == RED)
                    positionRedBlue(5.5, 2.875, alliance);
                else
                    positionRedBlue(5.5, 2.37, alliance);
                putBeaconPusherInPosition(3);
                doDaSleep(200);
                putBeaconPusherInPosition(2);
            }
        }
        else if(beaconsToGet[1])
        {
            positionRedBlue(4.5, 1.5, alliance);
        }

        if(beaconsToGet[1])
        {
            if(alliance == RED)
            {
                positionRedBlue(5.5, 4.62, alliance);
                rampTurnGyro(180);
            }
            else
            {
                positionRedBlue(5.5, 4.12, alliance);
                rampTurnGyro(0);
            }
            putBeaconPusherInPosition(2);
            if(isColorRed() == alliance)
            {
                putBeaconPusherInPosition(3);
                doDaSleep(200);
                putBeaconPusherInPosition(2);
            }
            else
            {
                if(alliance == RED)
                    positionRedBlue(5.5, 4.875, alliance);
                else
                    positionRedBlue(5.5, 4.37, alliance);
                putBeaconPusherInPosition(3);
                doDaSleep(200);
                putBeaconPusherInPosition(2);
            }
        }

        if(!shootBeforeBeacons && delayBeforeShooting < 30000)
        {
            doDaSleep(delayBeforeShooting);

            for(int i = 0; i < movementsToShoot.size(); i++)
            {
                moveToPosition(movementsToShoot.get(i)[0], movementsToShoot.get(i)[1]);
            }

            if(alliance == RED)
            {
                rampTurnGyro(angleToTurnToLocation(2.5, 2.5));
            }
            else
            {
                rampTurnGyro(angleToTurnToLocation(3.5, 2.5));
            }

            // Run flywheel and shoot
            targetFlywheelSpeed = 0.25;
            while(targetFlywheelSpeed != flywheelSpeed)
            {
                doDaSleep(10);
            }
            frontIntakesSpeed = 0.01;
            backIntakeSpeed = 0.99;
            doDaSleep(5000);
            frontIntakesSpeed = 0.5;
            backIntakeSpeed= 0.5;
            targetFlywheelSpeed = 0.0;
            while(targetFlywheelSpeed != flywheelSpeed)
            {
                doDaSleep(10);
            }
        }

        if(defense)
        {
            doDaSleep(delayBeforeDefense);
            for(int i = 0; i < movementsForDefense.size(); i++)
            {
                moveToPosition(movementsForDefense.get(i)[0], movementsForDefense.get(i)[1]);
            }
        }
        else if(endOnCornerVortex)
        {
            positionRedBlue(5.5, 2, alliance);
            rampTurnGyro(180);
            encoderDrives(MINIMUM_WHEEL_POWER_TO_MOVE, (int)(ENCODER_TICKS_TO_MOVE_ONE_TILE / 10), FORWARD);
        }

    }

    boolean colorisRed = false;

    void redAuton()
    {
        encoderDrives(0.1, 130, BACKWARD);
        encoderDrives(0.25, 130, BACKWARD);
        encoderDrives(0.4, 195, BACKWARD);
        encoderDrives(0.25, 130, BACKWARD);
        encoderDrives(0.1, 130, BACKWARD);
        gyroTurns(0.1, -11);
        rightWheelSpeed = 0.0;
        leftWheelSpeed = 0.0;
        doDaSleep(1000);
        rightWheelSpeed = 0.1;
        leftWheelSpeed = 0.1;
        doDaSleep(100);
        rightWheelSpeed = 0.0;
        leftWheelSpeed = 0.0;
        long daRedTimeVar = System.currentTimeMillis();
        if(gyro.getIntegratedZValue() < -48)
        {
            gyroTurns(0.15, -47);
        }
        else if(gyro.getIntegratedZValue() > -42)
        {
            while(gyro.getIntegratedZValue() > -42 && System.currentTimeMillis() < (daRedTimeVar + 2000))
            {
                leftWheelSpeed = 0.1;
                rightWheelSpeed = -0.1;
            }
        }
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        timerVar2 = System.currentTimeMillis();
        beaconPusherSpeed = 0.01;
        while((color.blue() < 3 && color.red() < 3) && (System.currentTimeMillis() > (timerVar2 + 800)))
        {
            doDaSleep(10);
        }
        if(color.blue() > 2 || color.red() > 2)
        {
            doDaSleep(400);
        }
        beaconPusherSpeed = 0.5;
        Drive_CorrectRed1.start();
        donePushing = false;
        do
        {
            leftWheelSpeed = 0.1;
            rightWheelSpeed = 0.1;
        } while(color.red() < 2);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(500);
        do
        {
            leftWheelSpeed = -0.1;
            rightWheelSpeed = -0.1;
        } while(color.red() < 2);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        Push_Beacon.start();
        while(!donePushing)
        {
            doDaSleep(10);
        }
        doDaSleep(200);

        encoderDrives(0.1, 150, BACKWARD);
        if(gyro.getIntegratedZValue() < -48)
        {
            if(gyro.getIntegratedZValue() < -54)
            {
                gyroTurns(0.15, -52);
            }
            gyroTurns(0.15, -47);
        }
        else if(gyro.getIntegratedZValue() > -42)
        {
            gyroTurns(0.1, -42);
        }
        leftWheelSpeed = 0.0; //Sam was here. You were terrible. I hope you won connor!
        rightWheelSpeed = 0.0;
        doDaSleep(350);
        donePushing = false;
        Drive_CorrectRed.start();
        encoderDrives(0.1, 50, BACKWARD);
        encoderDrives(0.25, 50, BACKWARD);
        encoderDrives(0.4, 40, BACKWARD);
        encoderDrives(0.25, 50, BACKWARD);
        encoderDrives(0.1, 50, BACKWARD);
        leftWheelSpeed = 0.1;
        rightWheelSpeed = 0.1;
        doDaSleep(500);
        do
        {
            leftWheelSpeed = 0.1;
            rightWheelSpeed = 0.1;
        } while(color.red() < 2);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(500);
        do
        {
            leftWheelSpeed = -0.07;
            rightWheelSpeed = -0.07;
        } while(color.red() < 2);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        Push_Beacon2.start();
        while(!donePushing)
        {
            doDaSleep(10);
        }

        doDaSleep(500);

        leftSpeedMultiplier = 1.4;
        gyroTurns(0.3, -31);
        gyroTurns(0.15, -22);
        leftSpeedMultiplier = 1.5;
        encoderDrives(0.1, 80, FORWARD);
        encoderDrives(0.25, 500, FORWARD);
        encoderDrives(0.1, 80, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        while(flywheelSpeed < 0.27)
        {
            targetFlywheelSpeed += 0.01;
            flywheelSpeed += 0.01;
            targetFlywheelSpeed = (double)Math.round(targetFlywheelSpeed * 100d) / 100d;
            flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
            doDaSleep(60);
        }
        doDaSleep(1000);
        RPM.start();
        backIntakeSpeed = 0.1;
        frontIntakesSpeed = 0.01;
        doDaSleep(3000);
        keepRunning = false;
        doDaSleep(100);
        backIntakeSpeed = 0.5;
        frontIntakesSpeed = 0.5;
        Steven_Ramp_Down.start();
        encoderDrives(0.15, 500, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        while(flywheelSpeed > 0.0)
        {
            doDaSleep(10);
        }
    }


    void blueAuton()
    {
        if((false == true) || (0 == 1)){
            return;
        }
        encoderDrives(0.1, 130, FORWARD);
        encoderDrives(0.25, 130, FORWARD);
        encoderDrives(0.4, 240, FORWARD);
        encoderDrives(0.25, 130, FORWARD);
        encoderDrives(0.1, 130, FORWARD);
        gyroTurns(0.1, 11);
        rightWheelSpeed = 0.0;
        leftWheelSpeed = 0.0;
        doDaSleep(500);
        rightWheelSpeed = 0.1;
        leftWheelSpeed = 0.1;
        doDaSleep(100);
        rightWheelSpeed = 0.0;
        leftWheelSpeed = 0.0;
        long daTimeVar = System.currentTimeMillis();
        if(gyro.getIntegratedZValue() < 38)
        {
                while(gyro.getIntegratedZValue() < 41 && System.currentTimeMillis() < (daTimeVar + 2000))
                {
                    leftWheelSpeed = -0.1;
                    rightWheelSpeed = 0.1;
                }
        }
        else if(gyro.getIntegratedZValue() > 49)
        {
            gyroTurns(0.15, 48);
        }
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        timerVar2 = System.currentTimeMillis();
        beaconPusherSpeed = 0.01;
        while((color.blue() < 3 && color.red() < 3) && (System.currentTimeMillis() > (timerVar2 + 800)))
        {
            doDaSleep(10);
        }
        if(color.blue() > 2 || color.red() > 2)
        {
            doDaSleep(400);
        }
        beaconPusherSpeed = 0.5;
        do
        {
            leftWheelSpeed = -0.08;
            rightWheelSpeed = -0.08;
        } while(color.blue() < 3);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(400);
        do
        {
            leftWheelSpeed = 0.08;
            rightWheelSpeed = 0.08;
        } while(color.blue() < 3);
        Push_Beacon.start();
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        while(!donePushing) {
            doDaSleep(10);
        }
        doDaSleep(200);
        donePushing = false;
        encoderDrives(0.1, 150, FORWARD);
        if(gyro.getIntegratedZValue() > 49)
        {
            if(gyro.getIntegratedZValue() > 52)
            {
                gyroTurns(0.12, 51);
            }
            else if(gyro.getIntegratedZValue() > 51)
            {
                gyroTurns(0.12, 51);
            }
            else
            {
                gyroTurns(0.12, 49);
            }
        }
        else if(gyro.getIntegratedZValue() < 46)
        {
            gyroTurns(0.12, 46);
        }
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(300);
        Drive_Correct.start();
        encoderDrives(0.1, 20, FORWARD);
        encoderDrives(0.3, 30, FORWARD);
        encoderDrives(0.5, 30, FORWARD);
        encoderDrives(0.6, 40, FORWARD);
        encoderDrives(0.3, 20, FORWARD);
        encoderDrives(0.1, 30, FORWARD);
        donePushing = true;
        leftWheelSpeed = -0.1;
        rightWheelSpeed = -0.1;
        doDaSleep(500);
        long timerStop = System.currentTimeMillis();
        do
        {
            leftWheelSpeed = -0.1;
            rightWheelSpeed = -0.1;
        } while(color.blue() < 3 && System.currentTimeMillis() < (timerStop + 4000));
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        if(System.currentTimeMillis() > (timerStop + 4000))
        {
            doDaSleep(20000);
        }
        else
        {
            doDaSleep(500);
        }
        do
        {
            leftWheelSpeed = 0.1;
            rightWheelSpeed = 0.1;
        } while(color.blue() < 3);
        donePushing = false;
        Push_Beacon2.start();
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        while(!donePushing)
        {
            doDaSleep(10);
        }

        doDaSleep(500);

        leftSpeedMultiplier = 1.4;
        gyroTurns(0.2, 60);
        gyroTurns(0.3, 125);
        gyroTurns(0.2, 170);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        doDaSleep(200);
        if(gyro.getIntegratedZValue() > 191)
        {
            gyroTurns(0.15, 191);
        }
        else if(gyro.getIntegratedZValue() < 185)
        {
            gyroTurns(0.15, 185);
        }
        leftSpeedMultiplier = 1.0;
        donePushing = false;
        Drive_Correct2.start();
        encoderDrives(0.1, 80, FORWARD);
        encoderDrives(0.25, 300, FORWARD);
        encoderDrives(0.1, 80, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        donePushing = true;
        while(flywheelSpeed < 0.27)
        {
            targetFlywheelSpeed += 0.01;
            flywheelSpeed += 0.01;
            targetFlywheelSpeed = (double)Math.round(targetFlywheelSpeed * 100d) / 100d;
            flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
            doDaSleep(60);
        }
        doDaSleep(1000);
        RPM.start();
        backIntakeSpeed = 0.1;
        frontIntakesSpeed = 0.01;
        doDaSleep(3000);
        keepRunning = false;
        doDaSleep(100);
        backIntakeSpeed = 0.5;
        frontIntakesSpeed = 0.5;
        Steven_Ramp_Down.start();
        encoderDrives(0.15, 500, FORWARD);
        while(flywheelSpeed > 0.0)
        {
            doDaSleep(10);
        }
    }

    long timeOfAuto = 0;

    void forwardLaunchAuton()
    {
        encoderDrives(0.1, 80, FORWARD);
        encoderDrives(0.25, 350, FORWARD);
        encoderDrives(0.1, 80, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        while(flywheelSpeed < 0.27)
        {
            targetFlywheelSpeed += 0.01;
            flywheelSpeed += 0.01;
            targetFlywheelSpeed = (double)Math.round(targetFlywheelSpeed * 100d) / 100d;
            flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
            doDaSleep(60);
        }
        doDaSleep(1000);
        RPM.start();
        backIntakeSpeed = 0.1;
        frontIntakesSpeed = 0.01;
        doDaSleep(2000);
        keepRunning = false;
        doDaSleep(100);
        backIntakeSpeed = 0.5;
        frontIntakesSpeed = 0.5;
        Steven_Ramp_Down.start();
        //encoderDrives(0.15, 500, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        if(daBlueAuto == false)
        {
            while (flywheelSpeed > 0.0) {
                doDaSleep(10);
            }
        }
        else {
            while (flywheelSpeed > 0.09) {
                doDaSleep(10);
            }
            blockAfterLaunch();
        }
    }

    void blockAfterLaunch()
    {
        encoderDrives(0.1, 80, FORWARD);
        encoderDrives(0.6, 150, FORWARD);
        encoderDrives(0.9, 1000, FORWARD);
        encoderDrives(0.6, 150, FORWARD);
        encoderDrives(0.1, 80, FORWARD);
        while(System.currentTimeMillis() < (timeOfAuto + 9000))
        {
            doDaSleep(10);
        }
        if(daBlueAuto) {
            gyroTurns(0.15, 15);
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
            doDaSleep(400);
        }
        else
        {
            gyroTurns(0.15, -45);
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
            doDaSleep(400);
            if (gyro.getIntegratedZValue() > -43) {
                gyroTurns(0.15, -43);
            }
        }
        encoderDrives(0.1, 80, FORWARD);
        encoderDrives(0.25, 600, FORWARD);
        encoderDrives(0.1, 80, FORWARD);
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
    }

    Runnable stevenRampDown = new Runnable() {
        public void run() {
            while(flywheelSpeed > 0.0)
            {
                targetFlywheelSpeed -= 0.01;
                flywheelSpeed -= 0.01;
                targetFlywheelSpeed = (double)Math.round(targetFlywheelSpeed * 100d) / 100d;
                flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
                doDaSleep(60);
            }
        }
    };

    Thread Steven_Ramp_Down = new Thread(stevenRampDown);

    Runnable pushBeacon = new Runnable()
    {
        public void run()
        {
            donePushing = false;
            beaconPusherSpeed = 0.01;
            doDaSleep(850);
            beaconPusherSpeed = 0.5;
            doDaSleep(500);
            beaconPusherSpeed = 0.99;
            doDaSleep(200);
            beaconPusherSpeed = 0.5;
            donePushing = true;
        }
    };


    Runnable pushBeacon2 = new Runnable()
    {
        public void run()
        {
            doDaSleep(700);
            donePushing = false;
            beaconPusherSpeed = 0.01;
            doDaSleep(800);
            beaconPusherSpeed = 0.5;
            doDaSleep(500);
            beaconPusherSpeed = 0.99;
            doDaSleep(600);
            beaconPusherSpeed = 0.5;
            donePushing = true;
        }
    };

    Thread Push_Beacon = new Thread(pushBeacon);
    Thread Push_Beacon2 = new Thread(pushBeacon2);

    Runnable correctDrive = new Runnable()
    {
        public void run()
        {
            while(donePushing == false)
            {
                if (gyro.getIntegratedZValue() < 45)
                {
                        leftSpeedMultiplier = 0.65;
                }
                else if (gyro.getIntegratedZValue() > 50)
                {
                        leftSpeedMultiplier = 4;
                }
                else if(gyro.getIntegratedZValue() > 48)
                {
                    leftSpeedMultiplier = 1.45;
                }
                else
                {
                    leftSpeedMultiplier = 1.25;
                }
            }
            leftSpeedMultiplier = 1.25;
        }
    };

    Runnable correctDriveVortex = new Runnable()
    {
        public void run()
        {
            while(donePushing == false)
            {
                if (gyro.getIntegratedZValue() < 188)
                {
                    leftSpeedMultiplier = 0.65;
                }
                else if (gyro.getIntegratedZValue() > 190)
                {
                    leftSpeedMultiplier = 4;
                }
                else
                {
                    leftSpeedMultiplier = 1.4;
                }
            }
            leftSpeedMultiplier = 1.4;
        }
    };

    Runnable correctDriveRed = new Runnable()
    {
        public void run()
        {
            while(donePushing == false)
            {
                if (gyro.getIntegratedZValue() < -48)
                {
                    leftSpeedMultiplier = 2.6;
                }
                else if (gyro.getIntegratedZValue() > -42)
                {
                    leftSpeedMultiplier = 0.8;
                }
                else
                {
                    leftSpeedMultiplier = 1.5;
                }
            }
            leftSpeedMultiplier = 1.5;
        }
    };

    Thread Drive_Correct = new Thread(correctDrive);
    Thread Drive_Correct2 = new Thread(correctDriveVortex);
    Thread Drive_CorrectRed = new Thread(correctDriveRed);
    Thread Drive_CorrectRed1 = new Thread(correctDriveRed);

    /*
     * AutoPro - The thread to run the autonomous that sets the motor powers
     *           and lets the main loop set the motors to those speeds, doing
     *           this because this makes more sense to me than state machines
     */
    Runnable AutoPro = new Runnable() {
        public void run() {
            // Run autonomous
            /*allAutonomous(currentAlliance, currentStartingCoordinates, currentDelay,
                          currentBeaconsToGet, currentShootBeforeBeacons, currentDefense,
                          currentDelayBeforeDefense, currentMovementsForDefense, currentDelayBeforeShooting,
                          currentMovementsToShoot, currentEndOnCornerVortex, currentSlowAutonomous);*/

            //driveTiles(1.0, FORWARD);
            doDaSleep(daAutoDelay);
            leftWheelMultiplier = 1.25;
            timeOfAuto = System.currentTimeMillis();
            while(gyro.getIntegratedZValue() != 0)
            {
                gyro.calibrate();

                while (gyro.isCalibrating()) {
                    doDaSleep(GYRO_CALIBRATION_WAIT_TIME);
                }

                doDaSleep(100);
            }
            if(launchForwardAuto)
            {
                forwardLaunchAuton();
            }
            else
            {
                if (daBlueAuto)
                {
                    blueAuton();
                }
                else
                {
                    redAuton();
                }
            }




            // At the end of the autonomous routine stop motors
            rightWheelSpeed = STOP_MOTOR_SPEED;
            leftWheelSpeed = STOP_MOTOR_SPEED;
            flywheelSpeed = 0.0;
            frontIntakesSpeed = 0.5;
            backIntakeSpeed = 0.5;
            beaconPusherSpeed = 0.5;
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

        ULFW = hardwareMap.dcMotor.get("URFW"); // Flywheel
        LRFW = hardwareMap.dcMotor.get("IRD");
        URFW = hardwareMap.dcMotor.get("ULFW");
        LLFW = hardwareMap.dcMotor.get("ILD");

        //FI = hardwareMap.servo.get("FI");
        BI = hardwareMap.servo.get("BI");
        BFI = hardwareMap.servo.get("BFI");
        FFI = hardwareMap.servo.get("FFI");
        BP = hardwareMap.servo.get("BP");


        color = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse Motors
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
        FRW.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reverse Flywheels
        LLFW.setDirection(DcMotorSimple.Direction.REVERSE);

        ULFW.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize all values of the parameters string array
        for(int h = 0; h < allParametersStringArray.length; h++)
        {
            allParametersStringArray[h] = "";
        }

        initializeAllSensorsSlashEncoders();
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        // Bumpers change the header and set the pre-set autonomous routines


        if(gamepad1.b)
        {
            daBlueAuto = false;
        }
        else if(gamepad1.x)
        {
            daBlueAuto = true;
        }

        if(gamepad1.dpad_up)
        {
            launchForwardAuto = true;
        }
        else if(gamepad1.dpad_down)
        {
            launchForwardAuto = false;
        }

        if(gamepad1.y && !pressedDaChange)
        {
            daAutoDelay += 1000;
            pressedDaChange = true;
        }
        else if(gamepad1.a && !pressedDaChange)
        {
            daAutoDelay -= 1000;
            pressedDaChange = true;
        }
        else if(gamepad1.y || gamepad1.a)
        {
            pressedDaChange = true;
        }
        else
        {
            pressedDaChange = false;
        }



        telemetry.addData("Delay", (daAutoDelay / 1000));
        telemetry.addData("Blue Auto? ", daBlueAuto);
        telemetry.addData("Just Launch? ", launchForwardAuto);





        /*if(gamepad1.left_bumper && headerStringArrayLocation > 0)
        {
            headerStringArrayLocation--;
            while(gamepad1.left_bumper)
            {

            }
            changedHeader = true;
        }
        if(gamepad1.right_bumper && headerStringArrayLocation < (headerStringArray.length - 1))
        {
            headerStringArrayLocation++;
            while(gamepad1.right_bumper)
            {

            }
            changedHeader = true;
        }

        // Switch case to change the values if pre-set autonomous
        if(changedHeader)
        {
            switch(headerStringArrayLocation)
            {
                case 0:
                    currentAlliance = true;
                    currentStartingCoordinates[0] = 4.5;
                    currentStartingCoordinates[1] = 0.5;
                    currentDelay = 0;
                    currentBeaconsToGet[0] = true;
                    currentBeaconsToGet[1] = true;
                    currentShootBeforeBeacons = true;
                    currentDefense = false;
                    currentDelayBeforeDefense = 0;
                    currentMovementsForDefense.clear();
                    currentDelayBeforeShooting = 0;
                    currentMovementsToShoot.clear();
                    currentMovementsToShoot.add(new double[]{1.5, 3.5});
                    currentEndOnCornerVortex = false;
                    currentSlowAutonomous = true;
                    break;
                case 1:
                    currentAlliance = false;
                    currentStartingCoordinates[0] = 4.0;
                    currentStartingCoordinates[1] = 0.36458;
                    currentDelay = 0;
                    currentBeaconsToGet[0] = true;
                    currentBeaconsToGet[1] = true;
                    currentShootBeforeBeacons = false;
                    currentDefense = false;
                    currentDelayBeforeDefense = 0;
                    currentMovementsForDefense.clear();
                    currentDelayBeforeShooting = 0;
                    currentMovementsToShoot.clear();
                    currentMovementsToShoot.add(new double[]{4.5, 3.5});
                    currentEndOnCornerVortex = false;
                    currentSlowAutonomous = true;
                    break;
                case 2:
                    currentAlliance = true;
                    currentStartingCoordinates[0] = 0.0;
                    currentStartingCoordinates[1] = 0.0;
                    currentDelay = 0;
                    currentBeaconsToGet[0] = false;
                    currentBeaconsToGet[1] = false;
                    currentShootBeforeBeacons = false;
                    currentDefense = false;
                    currentDelayBeforeDefense = 0;
                    currentMovementsForDefense.clear();
                    currentDelayBeforeShooting = 0;
                    currentMovementsToShoot.clear();
                    currentEndOnCornerVortex = false;
                    currentSlowAutonomous = false;
                    break;
                default:

            }
            upAndDownLocation = 0;
            sideToSideLocation = 0;
            changedHeader = false;
        }

        // D-pad up and down to change the current value you are changing
        if(gamepad1.dpad_down && upAndDownLocation < (allParametersStringArray.length - 1))
        {
            upAndDownLocation++;
            sideToSideLocation = 0;
            while(gamepad1.dpad_down)
            {

            }
        }
        if(gamepad1.dpad_up && upAndDownLocation > 0)
        {
            upAndDownLocation--;
            sideToSideLocation = 0;
            while(gamepad1.dpad_up)
            {

            }
        }

        // Set up all the string to display
        for(int h = 0; h < allParametersStringArray.length; h++)
        {
            allParametersStringArray[h] = "";
        }
        allParametersStringArray[0] = currentAlliance + "";
        allParametersStringArray[1] = currentStartingCoordinates[0] + "," + currentStartingCoordinates[1];
        allParametersStringArray[2] = ((double)currentDelay / 1000) + "";
        allParametersStringArray[3] = currentBeaconsToGet[0] + "," + currentBeaconsToGet[1];
        allParametersStringArray[4]  = currentShootBeforeBeacons + "";
        allParametersStringArray[5] = currentDefense + "";
        allParametersStringArray[6] = ((double)currentDelayBeforeDefense / 1000) + "";
        if(currentMovementsForDefense.size() > 0)
        {
            for (int u = 0; u < currentMovementsForDefense.size(); u++)
            {
                currentMovementsForDefense.get(u)[0] = (double)Math.round(currentMovementsForDefense.get(u)[0] * 100d) / 100d;
                currentMovementsForDefense.get(u)[1] = (double)Math.round(currentMovementsForDefense.get(u)[1] * 100d) / 100d;
                allParametersStringArray[7] += "(" + currentMovementsForDefense.get(u)[0] + "," +
                        currentMovementsForDefense.get(u)[1] + ") ";
            }
        }
        allParametersStringArray[8] = ((double)currentDelayBeforeShooting / 1000) + "";
        if(currentMovementsToShoot.size() > 0)
        {
            for (int u = 0; u < currentMovementsToShoot.size(); u++)
            {
                currentMovementsToShoot.get(u)[0] = (double)Math.round(currentMovementsToShoot.get(u)[0] * 100d) / 100d;
                currentMovementsToShoot.get(u)[1] = (double)Math.round(currentMovementsToShoot.get(u)[1] * 100d) / 100d;
                allParametersStringArray[9] += "(" + currentMovementsToShoot.get(u)[0] + "," +
                        currentMovementsToShoot.get(u)[1] + ") ";
            }
        }
        allParametersStringArray[10] = currentEndOnCornerVortex + "";
        allParametersStringArray[11] = currentSlowAutonomous + "";

        // Switch case to change values based on which value you are on and what buttons you press
        // Also puts the "*" at the end of the values you are currently editing
        switch(upAndDownLocation)
        {
            case 0:
                if(gamepad1.y)
                {
                    currentAlliance = true;
                }
                if(gamepad1.a)
                {
                    currentAlliance = false;
                }
                allParametersStringArray[0]  = currentAlliance + "*";
                break;
            case 1:
                if(gamepad1.dpad_left && sideToSideLocation > 0)
                {
                    sideToSideLocation--;
                    while(gamepad1.dpad_left)
                    {

                    }
                }
                if(gamepad1.dpad_right && sideToSideLocation < 1)
                {
                    sideToSideLocation++;
                    while(gamepad1.dpad_right)
                    {

                    }
                }
                if(gamepad1.y)
                {
                    currentStartingCoordinates[sideToSideLocation] += 0.1;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentStartingCoordinates[sideToSideLocation] += 0.01;
                    while(gamepad1.x)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentStartingCoordinates[sideToSideLocation] -= 0.1;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentStartingCoordinates[sideToSideLocation] -= 0.01;
                    while(gamepad1.b)
                    {

                    }
                }
                if(sideToSideLocation == 0)
                    allParametersStringArray[1] = currentStartingCoordinates[0] + "*," + currentStartingCoordinates[1];
                else
                    allParametersStringArray[1] = currentStartingCoordinates[0] + "," + currentStartingCoordinates[1] + "*";
                break;
            case 2:
                if(gamepad1.y)
                {
                    currentDelay += 1000;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentDelay += 100;
                    while(gamepad1.x)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentDelay -= 1000;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentDelay -= 100;
                    while(gamepad1.b)
                    {

                    }
                }
                allParametersStringArray[2] = ((double)currentDelay / 1000) + "*";
                break;
            case 3:
                if(gamepad1.dpad_left && sideToSideLocation > 0)
                {
                    sideToSideLocation--;
                    while(gamepad1.dpad_left)
                    {

                    }
                }
                if(gamepad1.dpad_right && sideToSideLocation < 1)
                {
                    sideToSideLocation++;
                    while(gamepad1.dpad_right)
                    {

                    }
                }
                if(gamepad1.y)
                {
                    currentBeaconsToGet[sideToSideLocation] = true;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentBeaconsToGet[sideToSideLocation] = false;
                    while(gamepad1.a)
                    {

                    }
                }
                if(sideToSideLocation == 0)
                    allParametersStringArray[3] = currentBeaconsToGet[0] + "*," + currentBeaconsToGet[1];
                else
                    allParametersStringArray[3] = currentBeaconsToGet[0] + "," + currentBeaconsToGet[1] + "*";
                break;
            case 4:
                if(gamepad1.y)
                {
                    currentShootBeforeBeacons = true;
                }
                if(gamepad1.a)
                {
                    currentShootBeforeBeacons = false;
                }
                allParametersStringArray[4]  = currentShootBeforeBeacons + "*";
                break;
            case 5:
                if(gamepad1.y)
                {
                    currentDefense = true;
                }
                if(gamepad1.a)
                {
                    currentDefense = false;
                }
                allParametersStringArray[5] = currentDefense + "*";
                break;
            case 6:
                if(gamepad1.y)
                {
                    currentDelayBeforeDefense += 1000;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentDelayBeforeDefense += 100;
                    while(gamepad1.x)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentDelayBeforeDefense -= 1000;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentDelayBeforeDefense -= 100;
                    while(gamepad1.b)
                    {

                    }
                }
                allParametersStringArray[6] = ((double)currentDelayBeforeDefense / 1000) + "*";
                break;
            case 7:
                allParametersStringArray[7] = "";
                if(gamepad1.dpad_left && sideToSideLocation > 0)
                {
                    if(sideToSideLocation % 2 == 0 && currentMovementsForDefense.get(sideToSideLocation / 2)[0] == 0.0
                            && currentMovementsForDefense.get(sideToSideLocation / 2)[1] == 0.0)
                    {
                        currentMovementsForDefense.remove(sideToSideLocation / 2);
                    }
                    sideToSideLocation--;
                    while(gamepad1.dpad_left)
                    {

                    }
                }
                if(gamepad1.dpad_right)
                {
                    if(((sideToSideLocation + 1) == ((currentMovementsForDefense.size() * 2))) || currentMovementsForDefense.size() == 0)
                    {
                        currentMovementsForDefense.add(new double[]{0.0, 0.0});
                        sideToSideLocation = ((currentMovementsForDefense.size() * 2) - 2);
                    }
                    else
                    {
                        sideToSideLocation++;
                    }
                    while(gamepad1.dpad_right)
                    {

                    }
                }
                if(gamepad1.y)
                {
                    currentMovementsForDefense.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] += 1.0;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentMovementsForDefense.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] -= 1.0;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentMovementsForDefense.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] -= 0.1;
                    while(gamepad1.b)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentMovementsForDefense.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] += 0.1;
                    while(gamepad1.x)
                    {

                    }
                }
                if(currentMovementsForDefense.size() > 0)
                {
                    for (int u = 0; u < currentMovementsForDefense.size(); u++)
                    {
                        if (sideToSideLocation / 2 == u || currentMovementsForDefense.size() == 1)
                        {
                            if (sideToSideLocation % 2 == 0)
                            {
                                allParametersStringArray[7] += "(" + currentMovementsForDefense.get(u)[0] + "*," +
                                        currentMovementsForDefense.get(u)[1] + ") ";
                            } else
                            {
                                allParametersStringArray[7] += "(" + currentMovementsForDefense.get(u)[0] + "," +
                                        currentMovementsForDefense.get(u)[1] + "*) ";
                            }
                        }
                        else
                        {
                            allParametersStringArray[7] += "(" + currentMovementsForDefense.get(u)[0] + "," +
                                    currentMovementsForDefense.get(u)[1] + ") ";
                        }
                    }
                }
                break;
            case 8:
                if(gamepad1.y)
                {
                    currentDelayBeforeShooting += 1000;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentDelayBeforeShooting += 100;
                    while(gamepad1.x)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentDelayBeforeShooting -= 1000;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentDelayBeforeShooting -= 100;
                    while(gamepad1.b)
                    {

                    }
                }
                allParametersStringArray[8] = ((double)currentDelayBeforeShooting / 1000) + "*";
                break;
            case 9:
                allParametersStringArray[9] = "";
                if(gamepad1.dpad_left && sideToSideLocation > 0)
                {
                    if(sideToSideLocation % 2 == 0 && currentMovementsToShoot.get(sideToSideLocation / 2)[0] == 0.0
                            && currentMovementsToShoot.get(sideToSideLocation / 2)[1] == 0.0)
                    {
                        currentMovementsToShoot.remove(sideToSideLocation / 2);
                    }
                    sideToSideLocation--;
                    while(gamepad1.dpad_left)
                    {

                    }
                }
                if(gamepad1.dpad_right)
                {
                    if(((sideToSideLocation + 1) == ((currentMovementsToShoot.size() * 2))) || currentMovementsToShoot.size() == 0)
                    {
                        currentMovementsToShoot.add(new double[]{0.0, 0.0});
                    }
                    sideToSideLocation++;
                    while(gamepad1.dpad_right)
                    {

                    }
                }
                if(gamepad1.y)
                {
                    currentMovementsToShoot.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] += 1.0;
                    while(gamepad1.y)
                    {

                    }
                }
                if(gamepad1.a)
                {
                    currentMovementsToShoot.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] -= 1.0;
                    while(gamepad1.a)
                    {

                    }
                }
                if(gamepad1.b)
                {
                    currentMovementsToShoot.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] -= 0.1;
                    while(gamepad1.b)
                    {

                    }
                }
                if(gamepad1.x)
                {
                    currentMovementsToShoot.get((sideToSideLocation / 2))[(sideToSideLocation % 2)] += 0.1;
                    while(gamepad1.x)
                    {

                    }
                }
                if(currentMovementsToShoot.size() > 0)
                {
                    for (int u = 0; u < currentMovementsToShoot.size(); u++)
                    {
                        if (sideToSideLocation / 2 == u)
                        {
                            if (sideToSideLocation % 2 == 0)
                            {
                                allParametersStringArray[9] += "(" + currentMovementsToShoot.get(u)[0] + "*," +
                                        currentMovementsToShoot.get(u)[1] + ") ";
                            }
                            else
                            {
                                allParametersStringArray[9] += "(" + currentMovementsToShoot.get(u)[0] + "," +
                                        currentMovementsToShoot.get(u)[1] + "*) ";
                            }
                        }
                        else
                        {
                            allParametersStringArray[9] += "(" + currentMovementsToShoot.get(u)[0] + "," +
                                    currentMovementsToShoot.get(u)[1] + ") ";
                        }
                    }
                }
                break;
            case 10:
                if(gamepad1.y)
                {
                    currentEndOnCornerVortex = true;
                }
                if(gamepad1.a)
                {
                    currentEndOnCornerVortex = false;
                }
                allParametersStringArray[10] = currentEndOnCornerVortex + "*";
                break;
            case 11:
                if(gamepad1.y)
                {
                    currentSlowAutonomous = true;
                }
                if(gamepad1.a)
                {
                    currentSlowAutonomous = false;
                }
                allParametersStringArray[11] = currentSlowAutonomous + "*";
                break;
            default:
                break;
        }

        // Display the current autonomous configuration
        telemetry.addData("Heading ", headerStringArray[headerStringArrayLocation]);
        telemetry.addData("Alliance ", currentAlliance);
        telemetry.addData("Color Red ", color.red());
        telemetry.addData("Delay ", allParametersStringArray[2]);
        telemetry.addData("Color Blue ", color.blue());
        telemetry.addData("Shoot Before Beacons ", allParametersStringArray[4]);
        telemetry.addData("Defense " , allParametersStringArray[5]);
        telemetry.addData("Delay Before Defense ", allParametersStringArray[6]);
        telemetry.addData("Movements For Defense ", allParametersStringArray[7]);
        telemetry.addData("Delay Before Shooting ", allParametersStringArray[8]);
        telemetry.addData("Movements To Shoot ", allParametersStringArray[9]);
        telemetry.addData("End on Corner Vortex", allParametersStringArray[10]);
        telemetry.addData("Slow Autonomous", allParametersStringArray[11]);*/
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

        // Ramp the flywheel up and down
        if(targetFlywheelSpeed != flywheelSpeed && (System.currentTimeMillis() > (timerVar + 30)))
        {
            flywheelSpeed += (((targetFlywheelSpeed - flywheelSpeed) / (Math.abs(targetFlywheelSpeed - flywheelSpeed))) * 0.01);
            timerVar = System.currentTimeMillis();
            flywheelSpeed = (double)Math.round(flywheelSpeed * 100d) / 100d;
        }
        else if(targetFlywheelSpeed == flywheelSpeed)
        {
            timerVar = 0;
        }

        // Run the motors at the speeds defined by the autonomous thread
        runLeftWheels((leftWheelSpeed * leftSpeedMultiplier));
        runRightWheels(rightWheelSpeed);
        runFlywheel(-flywheelSpeed);
        runFrontIntakes(frontIntakesSpeed);
        BI.setPosition(backIntakeSpeed);
        BP.setPosition(beaconPusherSpeed);

        telemetry.addData("Right Speed", rightWheelSpeed);
        telemetry.addData("Left Speed", leftWheelSpeed);
        telemetry.addData("Gyro ", gyro.getIntegratedZValue());
        telemetry.addData("Right Encoder", getRightWheelEncoderValue());
        telemetry.addData("Left Encoder Pure Value", leftEncoderMotor().getCurrentPosition());
        telemetry.addData("Red", color.red());
        telemetry.addData("Flywheel Encoder", flywheelEncoderSpeed);
        telemetry.addData("Flywheel Speed", flywheelSpeed);

        doDaSleep(4);
    }

}
