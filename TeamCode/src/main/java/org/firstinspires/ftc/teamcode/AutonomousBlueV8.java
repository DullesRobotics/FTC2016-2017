package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/13/2017.
 */

@Autonomous(name = "[BLUE] Autonomous V8")

public class AutonomousBlueV8 extends LinearVisionOpMode {
    AutonomousDriveClassV3 autonomous;
    AdvancedRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AdvancedRobot(this);
        autonomous = new AutonomousDriveClassV3(robot);
        waitForStart();

        autonomous.driveSetTime(3, AutonomousDriveClassV3.Direction.FORWARD);
    }
}
