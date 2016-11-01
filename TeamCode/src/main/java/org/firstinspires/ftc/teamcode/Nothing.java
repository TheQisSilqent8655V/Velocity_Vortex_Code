package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * Created by Conno on 10/29/2016.
 * */
@TeleOp(name = "Init loop test", group = "Autonomous Tests")
public class Nothing extends OpMode{

    boolean currentAlliance = false;
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

    // Variables for displaying information in init loop
    String[] headerStringArray = {"Red 2 Beacon Shoot Basic", "Blue 2 Beacon Shoot Basic", "Custom"}; // Variables that change with buttons
    int headerStringArrayLocation = 0;
    int sideToSideLocation = 0;
    int upAndDownLocation = 1;
    boolean changedHeader = true;

    String[] allParametersStringArray = new String[11];

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        //currentMovementsForDefense.add(fillerDoubleArray);
        //currentMovementsToShoot.add(fillerDoubleArray);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        // Bumpers change the header and set the pre-set autonomous routines
        if(gamepad1.left_bumper && headerStringArrayLocation > 0)
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
                    currentStartingCoordinates[0] = 2.0;
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
                    currentMovementsToShoot.add(new double[]{1.5, 3.5});
                    currentEndOnCornerVortex = false;
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
            default:
                break;
        }

        // Display the current autonomous configuration
        telemetry.addData("Pre-Set Autonomous ", headerStringArray[headerStringArrayLocation]);
        telemetry.addData("Red Alliance ", allParametersStringArray[0]);
        telemetry.addData("Starting Coordinates ", allParametersStringArray[1]);
        telemetry.addData("Delay ", allParametersStringArray[2]);
        telemetry.addData("Beacons To Get ", allParametersStringArray[3]);
        telemetry.addData("Shoot Before Beacons ", allParametersStringArray[4]);
        telemetry.addData("Defense " , allParametersStringArray[5]);
        telemetry.addData("Delay Before Defense ", allParametersStringArray[6]);
        telemetry.addData("Movements For Defense ", allParametersStringArray[7]);
        telemetry.addData("Delay Before Shooting ", allParametersStringArray[8]);
        telemetry.addData("Movements To Shoot ", allParametersStringArray[9]);
        telemetry.addData("End on Corner Vortex", allParametersStringArray[10]);
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

    }

}
