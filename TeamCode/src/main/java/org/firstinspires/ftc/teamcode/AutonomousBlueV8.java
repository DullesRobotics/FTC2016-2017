package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/13/2017.
 *
 * CONCEPT FOR AUTONOMOUS:
 * 1) Move robot forward slightly
 * 2) Strafe to the right slightly, this way the center isnt in our way at all
 * 3) Move forward some more so the corner vertex is no longer next to us
 * 4) Turn 90 degrees to the right to face wall
 * 5) Move forward till we're roughly half a foot from the front wall
 * 6) Strife left till we hit the line
 * 7) Read beacon and push
 * 8) Repeat step 6 - 7
 * 9) Hopefully we've hit both beacons by now
 * --------------------------------------------------------------------------
 * 10) Back robot up
 * 11) Go full force toward the center and park there
 *
 * or
 *
 * 10) Back robot up
 * 11) Go semi-full force toward the corner and park there
 *
 * or
 *
 * 10) Back robot up
 * 11) Go full force toward center, but park on corner
 *
 * 10-11, we should make separate opmodes for each.
 */

@Autonomous(name = "[BLUE] Autonomous V8")

public class AutonomousBlueV8 extends LinearVisionOpMode {
    AutonomousDriveClassV3 autonomous;
    AdvancedRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AdvancedRobot(this);
        autonomous = new AutonomousDriveClassV3(robot);

        double defaultSpeed = .75;
        //String TeamOn = "Blue";

        waitForStart();

        telemetry.update();

        /*
       autonomous.driveTillLine(AutonomousDriveClassV3.Direction.FORWARD,.5,10);
       */

        /*Start out sideways facing corner vortex
        Left for a few seconds
        Forward till sensor can touch line when moved left
        Left till line
        SetServo
        Forward
        Back
        Left for .5 seconds
        Left till line
        SetServo
        Forward
        Back
         */
    }
}
