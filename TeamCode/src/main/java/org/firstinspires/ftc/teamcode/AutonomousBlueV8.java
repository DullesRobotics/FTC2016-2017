package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/13/2017.
 *
 * TODO:
 * -Create AutonomousDriveClassV3
 * -Implement AutonomousDriveClassV3
 * -Attempt simple time-based autonomous?
 */

@Autonomous(name = "[BLUE] Autonomous V8")
public class AutonomousBlueV8 extends LinearVisionOpMode {
    AutonomousDriveClassV2 auto;
    FTCVisionManager vision;
    AdvancedRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        vision.initFTCVision();
        robot = new AdvancedRobot(this);
    }
}
