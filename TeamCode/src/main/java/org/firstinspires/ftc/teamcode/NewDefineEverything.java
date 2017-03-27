package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.locks.Lock;

/**
 * Created by Conno on 3/10/2017.
 */
public class NewDefineEverything extends OpMode {

    // Define Vex Motors
    Servo FRW;  // Drive
    Servo MRW;
    Servo BRW;
    Servo FLW;
    Servo MLW;
    Servo BLW;

    Servo FRFW; // Flywheel
    Servo BRFW;
    Servo FLFW;
    Servo BLFW;

    Servo Intake;
    Servo BeaconPusher;

    // Define AndyMark Motors
    DcMotor Lift;

    // Define Sensors
    ColorSensor Color;
    AdafruitBNO055IMU stevens_IMU;
    BNO055IMU.Parameters parameters;
    I2cDeviceSynch FlywheelEncoder;
    VEXEncoder Flywheel;

    // Variables for both autonomous and teleOp to use
    double rightWheelSpeed = 0.0;
    double leftWheelSpeed = 0.0;
    double targetFlywheelSpeed = 0.0;
    double currentFlywheelSpeed = 0.0;
    long timerVar = 0;
    boolean rampingDown = false;
    int error = 0;
    int fillerInt = 0;
    int flywheelEncoderSpeed = 0;
    int targetFlywheelEncoderSpeed = 0;
    int val = 0;
    final int FLYWHEEL_RAMP_TIME = 60;

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
        //FlywheelEncoder = hardwareMap.i2cDeviceSynch.get("F");
        //Flywheel = new VEXEncoder(0x10, FlywheelEncoder);
        Color = hardwareMap.colorSensor.get("color");
        stevens_IMU = new AdafruitBNO055IMU(hardwareMap.i2cDeviceSynch.get("IMU")); // Adafruit Gyro
        parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        boolean success = stevens_IMU.initialize(parameters);

        //telemetry.addData("Success: ", success);


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

    /**
     * Converts at percentage -1.0 to 1.0 to a
     * servo value that vex motors can use
     *
     * @param power the percent power to convert
     * @return      the value to run the servo at
     */
    double convertToServoPower(double power)
    {
        power = Range.clip(power, -1.0, 1.0);
        return (0.5 + (power * 0.49));
    }


    /**
     * Function to run the right drive wheels
     *
     * @param speed power to run the wheels at
     */
    void runRightWheels(double speed)
    {
        speed = convertToServoPower(speed);
        FRW.setPosition(speed);
        MRW.setPosition(speed);
        BRW.setPosition(speed);
    }

    /**
     * Function to run the left drive wheels
     *
     * @param speed power to run the wheels at
     */
    void runLeftWheels(double speed)
    {
        speed = convertToServoPower(speed);
        FLW.setPosition(speed);
        MLW.setPosition(speed);
        BLW.setPosition(speed);
    }

    /**
     * Runs the flywheel
     *
     * @param speed the power to run at
     */
    void runFlywheel(double speed)
    {
        speed = convertToServoPower(speed);
        FRFW.setPosition(speed);
        BRFW.setPosition(speed);
        FLFW.setPosition(speed);
        BLFW.setPosition(speed);
    }

    /**
     * Runs the intake
     *
     * @param speed - power to run at
     */
    void runIntake(double speed)
    {
        speed = convertToServoPower(speed);
        Intake.setPosition(speed);
    }

    /**
     * Runs the lift
     *
     * @param speed power to run at
     */
    void runLift(double speed)
    {
        speed = Range.clip(speed, -1.0, 1.0);
        Lift.setPower(speed);
    }

    /**
     * Runs the beacon pusher
     *
     * @param speed power to run at
     */
    void runBeaconPusher(double speed)
    {
        speed = convertToServoPower(speed);
        BeaconPusher.setPosition(speed);
    }

    /**
     * This ramps the speed of the flywheel to be nice to motors
     */
    void rampFlywheel()
    {
        if(targetFlywheelSpeed != currentFlywheelSpeed && (System.currentTimeMillis() > (timerVar + FLYWHEEL_RAMP_TIME)))
        {
            currentFlywheelSpeed += (((targetFlywheelSpeed - currentFlywheelSpeed) / (Math.abs(targetFlywheelSpeed - currentFlywheelSpeed))) * 0.01);
            timerVar = System.currentTimeMillis();
            currentFlywheelSpeed = (double)Math.round(currentFlywheelSpeed * 100d) / 100d;
        }
        else if(targetFlywheelSpeed == currentFlywheelSpeed)
        {
            timerVar = 0;
        }
    }

    /**
     * This makes the thread sleep for the entered amount of time
     *
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
