package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Conno on 1/14/2017.
 */
//@TeleOp(name = "TestBot TeleOp", group = "TeleOp")
public class TestBot extends OpMode{

    DcMotor LW;
    DcMotor RW;

    int driveDivider = 2;
    int mode = 1;
    boolean changedDrive = false;
    double rightPower = 0.0;
    double leftPower = 0.0;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        LW = hardwareMap.dcMotor.get("LW");
        RW = hardwareMap.dcMotor.get("RW");

        RW.setDirection(DcMotorSimple.Direction.REVERSE);

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

        if(gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y){
            if(changedDrive == false)
            {
                if (mode == 1) mode = 2;
                else if (mode == 2) mode = 1;
                changedDrive = true;
            }
        }
        else
        {
            changedDrive = false;
        }

        if(mode == 1) {
            telemetry.addData("Mode", "Tank Drive");

            if(gamepad1.right_bumper){
                rightPower = gamepad1.right_stick_y;
                leftPower = gamepad1.left_stick_y;
            }
            else{
                rightPower = (gamepad1.right_stick_y/2);
                leftPower = (gamepad1.left_stick_y/2);
            }
        }

        if(mode == 2){
            telemetry.addData("Mode", "Single-Jostick Drive");
            if((gamepad1.right_stick_x > 0.1 || gamepad1.right_stick_x < -0.1) || (gamepad1.right_stick_y > 0.1 || gamepad1.right_stick_y < -0.1)) {
                rightPower = Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -1.0, 1.0);
                leftPower = Range.clip(gamepad1.right_stick_y - gamepad1.right_stick_x, -1.0, 1.0);
            }
            else
            {
                rightPower = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x, -1.0, 1.0);
                leftPower = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -1.0, 1.0);
            }
        }

        RW.setPower(rightPower);
        LW.setPower(leftPower);

        telemetry.addData("Right ", rightPower);
        telemetry.addData("Left ", leftPower);
    }
}
