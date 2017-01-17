package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Conno on 9/22/2016.
 */
public class DefineEverything extends OpMode {

    // Define Motors
    DcMotor FRW; // Drive
    DcMotor BRW;
    DcMotor FLW;
    DcMotor BLW;

    DcMotor URFW; // Flywheel
    DcMotor LRFW;
    DcMotor ULFW;
    DcMotor LLFW;

    // Define Servos
    Servo BI;
    Servo FFI;
    Servo BFI;
    Servo BP;

    // Define Sensors
    ColorSensor color;
    ModernRoboticsI2cGyro gyro;

    // General Constants
    final double MAXIMUM_MOTOR_SPEED = 1.0;
    final double MINIMUM_MOTOR_SPEED = -1.0;
    final double NOTHING = 0.0;
    final double STOP_MOTOR_SPEED = 0.0;

    // Autonomous Constants
    final int BACKWARD = 1; // General
    final int FORWARD = -1;
    int OVERRUN_CORRECTION_WAIT_TIME = 200;
    final double MINIMUM_WHEEL_POWER_TO_MOVE = 0.3;
    final double RAMP_UP_START_SPEED = 0.2;
    final int TICKS_PER_WHEEL_MOTOR_ROTATION = 1120;
    final double WHEEL_GEAR_RATIO = 4;
    final double WHEEL_DIAMETER = 4;
    final double ENCODER_TICKS_TO_MOVE_ONE_TILE = (24 / (WHEEL_DIAMETER * Math.PI * WHEEL_GEAR_RATIO)) * TICKS_PER_WHEEL_MOTOR_ROTATION;
    final boolean RED = true;
    final boolean BLUE = false;

    final int ENCODER_CALIBRATION_WAIT_TIME = 100; // Initialize function
    final int GYRO_CALIBRATION_WAIT_TIME = 10;

    final int DIFFERENCE_IN_WHEEL_ENCODER_THRESHOLD = 50; // Basic encoder drives
    final double ENCODER_DRIVE_STRAIGHT_CORRECTION = 0.9;

    int RAMP_DOWN_ENCODER_STEP = 300; // Ramp encoder drive
    final int RAMP_ENCODER_STEPS = 3;
    int  RAMP_UP_ENCODER_STEP = 200;
    final int SINGLE_STEP_OVERRUN_ADJUSTMENT = 50;

    final int GYRO_RAMP_STEPS = 2; // Ramp gyro turning
    int GYRO_STEP_VALUE = 25;
    final int SINGLE_STEP_OVERTURN_ADJUSTMENT = 13;
    final double GYRO_TURN_MAX_SPEED = 0.8;

    final int STRAIGHT_ANGLE = 180; // Move to position
    final int POWER_OF_TWO = 2;

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


        // Map Sensors
        color = hardwareMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

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
        BLW.setPower((Range.clip((speed * 1.0), -1.0, 1.0)));
        LLFW.setPower(speed);
    }

    /*
     * runFlywheel - Function to run the flywheel
     * @param double speed - the power to run at
     */
    void runFlywheel(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        ULFW.setPower(speed);
        URFW.setPower(speed);
    }

    /*
     * runFrontIntakes - Function to run both front intakes
     * @param double speed - power to run at, 0.01 for backwards, 0.99 for forwards, 0.5 for stop
     */
    void runFrontIntakes(double speed)
    {
        speed = Range.clip(speed, 0.01, 0.99);
        BFI.setPosition(speed);
        FFI.setPosition(speed);
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

    /*
     * This is to make it easier to change which wheel has the encoder
     * @return - the motor with the encoder
     */
    DcMotor rightEncoderMotor()
    {
        return LRFW;
    }

    /*
     * This is to make it easier to change which wheel has the encoder
     * @return - the motor with the encoder
     */
    DcMotor leftEncoderMotor()
    {
        return ULFW;
    }

    /*
     * This is to make it easier to change which wheel has the encoder
     * @return - the encoder value
     */
    int getRightWheelEncoderValue()
    {
        return rightEncoderMotor().getCurrentPosition();
    }

    /*
     * This is to make it easier to change which wheel has the encoder
     * @return - the encoder value
     */
    int getLeftWheelEncoderValue()
    {
        return leftEncoderMotor().getCurrentPosition();
    }

}
