package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 5/19/2017.
 */

@Autonomous(name = "[BOTH] Forward w/ 20 delay")
public class ForwardAutonomous extends LinearVisionOpMode{
    AdvancedRobot robot;
    AutonomousDriveClassV3 autonomous;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AdvancedRobot(this);
        autonomous = new AutonomousDriveClassV3(robot);
        waitForStart();
        autonomous.delayAutonomous(20);
        autonomous.driveSetTime(3, AutonomousDriveClassV3.Direction.FORWARD,.5);
    }
}
