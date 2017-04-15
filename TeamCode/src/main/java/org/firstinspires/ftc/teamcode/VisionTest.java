package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 2/1/2017.
 */
import static com.dullesrobotics.ftc.libraries.commonMethods.delay;
@Autonomous(name = "Vision Test")
public class VisionTest extends LinearVisionOpMode {
    FTCVisionManager ftcVisionManager;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();

        //Initialize Robot
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        telemetry.addData("eakjg","afkj");
        while(opModeIsActive()){
            telemetry.addData("Status",ftcVisionManager.readBeacon(5,5.0));
            delay(1000);
        }
    }
}
