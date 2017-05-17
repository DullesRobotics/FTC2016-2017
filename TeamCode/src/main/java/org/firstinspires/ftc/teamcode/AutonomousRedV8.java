package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 5/17/2017.
 */

public class AutonomousRedV8 extends LinearVisionOpMode{
    AutonomousDriveClassV3 autonomous;
    AdvancedRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AdvancedRobot(this);
        autonomous = new AutonomousDriveClassV3(robot);

        waitForStart();

        /*Start out sideways facing corner vortex
        Right for a few seconds
        Forward till in line with line
        Right till line
        SetServo
        Forward
        Back
        Right .5 second
        Right till line
        SetServo
        Forward
        Back

         */
    }
}
