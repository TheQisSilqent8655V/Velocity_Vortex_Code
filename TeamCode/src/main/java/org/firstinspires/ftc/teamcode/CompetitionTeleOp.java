package org.firstinspires.ftc.teamcode;

/**
 * Created by Conno on 11/2/2016.
 */
public class CompetitionTeleOp extends DefineEverything {



    /*
     * controlFlywheel - Controls the flywheel based on the target speed changed by buttons by
     *                   ramping the speed up and down by a amount to be tuned later
     * Button Mapping - 2A: 0.0 speed, 2Y: 0.25 speed, 2X: -= 0.01 speed, 2B: += 0.01 speed
     */

    /*
     * controlDrive - Controls the drive with tank drive and turbo/slow options, default is half speed
     * Button Mapping - 1Right Stick Y: value = right side speed, 1Left Stick Y: value = left side speed
     *                  1Right Bumper: hold for full speed, 1Right Trigger: hold > 0.2 for quarter speed
     */

    /*
     * controlFrontIntakes - Controls the front 2 intakes(< 0.5 is intake, > 0.5 is outtake), default is stop
     * Button Mapping - 1Left Bumper: hold for intake, 1Left Trigger > 0.2: hold for outtake
     */

    /*
     * controlBackIntake - Controls the back intake based on the speed the flwheel is moving
     *                     outtake if intaking, intake is if intaking and flywheel > 0.0 and at target speed,
     *                     default to not moving
     */

    /*
     * controlBeaconPusher - Controls the arm to push the beacon
     * Button Mapping - 2Dpad Left: hold for move out, 2Dpad Right: hold for move in
     */




    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

    }
}
