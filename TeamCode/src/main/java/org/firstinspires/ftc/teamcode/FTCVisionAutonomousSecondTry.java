package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 1/7/2017.
 */

public class FTCVisionAutonomousSecondTry extends LinearVisionOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;

    int sleepTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();

        //Initialize Robot
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);

        //Sets Up Camera
        //initializes camera
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();


        //Allow Setting a time delay
        final int programedDelay = 10;
        final boolean overrideControls = false;
        final int MAXSLEEPTIME = 20;

        boolean prevStateBtnA = false;
        boolean prevStateBtnB = false;

        //You have 10 seconds to set the delay
        while(runtime.seconds() < 10 && opModeIsActive()) {
            if(gamepad1.a){
                sleepTime+=5;
                if (sleepTime > MAXSLEEPTIME){
                    sleepTime = MAXSLEEPTIME;
                }
            } else if (gamepad1.b){
                sleepTime-=5;
                if(sleepTime < 0){
                    sleepTime = 0;
                }
            }


            prevStateBtnA = gamepad1.a;
            prevStateBtnB = gamepad1.b;
        }

        waitForStart(); //Wait for START Button Press on DS
        delay(sleepTime*1000);




    }
}
