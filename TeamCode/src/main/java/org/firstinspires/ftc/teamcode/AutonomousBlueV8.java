package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

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

        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.RIGHT);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.LEFT);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD_RIGHT);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD_LEFT);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD_LEFT);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD_RIGHT);
    }
}
