package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.AutonomousDrive;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
@Autonomous(name = "AutonV4BLUE")
public class AutonomousBlueV4 extends LinearVisionOpMode {
    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;
    private ElapsedTime runtime = new ElapsedTime();
    RobotWithFlickerShooter robot;
    AutonomousDrive autonomousDrive;
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();

        //Initialize Robot
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDrive(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"));
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");

        //Sets Up Camera
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart(); //Wait for START Button Press on DS

        //START
        autonomousDrive.swingTurn(0.2, 37.0, 5.0);
        autonomousDrive.encoderDrive(0.4, 60.0, 60.0, 10.0);
        autonomousDrive.driveTillLine(0.2, 10.0, AutonomousDrive.EOPDWHITELINELIGHTLEVEL);
        autonomousDrive.encoderDrive(0.2, 5.0, 5.0, 5.0);
        autonomousDrive.turnTillLine(0.2, AutonomousDrive.EOPDWHITELINELIGHTLEVEL, false);
        autonomousDrive.encoderDrive(0.4, -30.0, -30.0, 8.0);



        /*End Manuver*/
        //BEGIN read beacon
        telemetry.addData("Action", "Begin read beacon");
        String beaconAnalysis = ftcVisionManager.readBeacon(5, 5.0);

        if (beaconAnalysis.equals("redBlue")) {
            //Press Right side b/c we are blue
            telemetry.addData("Analysis", "Push right");
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            //Turn Servo

        } else {
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);//Turn Servo
            telemetry.addData("Analysis", "Push left");
        }
        //Drive Forwards to press with lower power, keep pushing for some time
        autonomousDrive.encoderDrive(0.2, 15.0, 15.0, 8.0);
        autonomousDrive.followLine(0.2, AutonomousDrive.EOPDWHITELINELIGHTLEVEL, 4.0, false);
        //Backup, go to other beacon
        autonomousDrive.encoderDrive(0.4, -40.0, -40.0, 5.0);
        autonomousDrive.pointTurn(0.2, -100.0, 5.0);
        autonomousDrive.driveTillLine(0.4, 8.0, AutonomousDrive.EOPDWHITELINELIGHTLEVEL);
        autonomousDrive.turnTillLine(0.2,AutonomousDrive.EOPDWHITELINELIGHTLEVEL,false);
        autonomousDrive.encoderDrive(0.4,-10.0,-10.0,5.0);

        String secondBeaconAnalysis = ftcVisionManager.readBeacon(5,10.0);
        if (beaconAnalysis.equals("redBlue")) {
            //Press Right side b/c we are blue
            telemetry.addData("Analysis", "Push right");
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            //Turn Servo

        } else {
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);//Turn Servo
            telemetry.addData("Analysis", "Push left");
        }
        autonomousDrive.encoderDrive(0.2, 15.0, 15.0, 8.0);
        autonomousDrive.followLine(0.2, AutonomousDrive.EOPDWHITELINELIGHTLEVEL, 4.0, false);
    }



}
