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
        double defaultSpeed = .75;
        waitForStart();

        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.RIGHT,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.LEFT,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD_RIGHT,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD_LEFT,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.FORWARD_LEFT,defaultSpeed);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.BACKWARD_RIGHT,defaultSpeed);
    }
}
