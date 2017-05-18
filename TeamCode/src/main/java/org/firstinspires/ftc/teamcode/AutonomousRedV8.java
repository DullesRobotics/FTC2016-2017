package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 5/17/2017.
 */

@Autonomous(name="[RED] Autonomous V8")
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
        autonomous.driveSetTime(2.2, AutonomousDriveClassV3.Direction.RIGHT,0.4);
        autonomous.driveSetTime(2.2, AutonomousDriveClassV3.Direction.FORWARD,0.2);
        autonomous.driveTillLine(AutonomousDriveClassV3.Direction.RIGHT,0.2, 5.0);
        autonomous.driveSetTime(0.23101, AutonomousDriveClassV3.Direction.RIGHT,0.1501);
        autonomous.setServo("red");
        autonomous.driveSetTime(2.3, AutonomousDriveClassV3.Direction.FORWARD, 0.2);
        //ADD TODO here
        autonomous.driveSetTime(1.2, AutonomousDriveClassV3.Direction.BACKWARD, 0.25);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.RIGHT, 0.2);
        autonomous.driveTillLine(AutonomousDriveClassV3.Direction.RIGHT, 0.3, 5.0);
        autonomous.setServo("red");
        autonomous.driveSetTime(2.0, AutonomousDriveClassV3.Direction.FORWARD, 0.3);
        autonomous.driveSetTime(2.0, AutonomousDriveClassV3.Direction.BACKWARD, 0.3);
    }
}
