package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
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
    AutonomousDriveClass autonomousDrive;
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;
    final boolean DEBUG = true;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        //this.resetStartTime();
        debug(1);
        //Initialize Robot
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDriveClass(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"));
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive.resetEncoders();
        debug(2);
        //Sets Up Camera
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart(); //Wait for START Button Press on DS
        debug(3);
        //START
        autonomousDrive.swingTurn(0.35, 30.0, 2.5); //Power, degree, timeout
        debug(4);
        //autonomousDrive.encoderDrive(0.4, 0.0, 0.0, 10.0); //Speed, Left, Right, timeout
        debug(5);
        autonomousDrive.driveTillLine(.4, 10.0, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL);//Power threshold, turnLeft
        debug(6);
        autonomousDrive.encoderDrive(0.2, 2.5, 2.5, 5.0);
        debug(7);
        autonomousDrive.swingTurn(.35,40,3.0);
        debug(7.1);
        autonomousDrive.encoderDrive(.2,-5,-5,5.0);
        debug(7.2);
        autonomousDrive.driveTillLine(.4,10.0,AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL);
        debug(7.3);
        autonomousDrive.turnLeftTillLine(0.2, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL, false);
        debug(8);
        autonomousDrive.encoderDrive(0.4, -5.0, -5.0, 5.0);
        debug(9);


        /*End Manuver*/
        //BEGIN read beacon
        telemetry.addData("Action", "Begin read beacon");
        String beaconAnalysis = ftcVisionManager.readBeacon(5, 5.0);
        debug(10);

        if (beaconAnalysis.equals("redBlue")) {
            //Press Right side b/c we are blue
            debug(11);
            telemetry.addData("Analysis", "Push right");
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            //Turn Servo
        } else {
            //Press Left side b/c we are blue
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);//Turn Servo
            debug(12);
            telemetry.addData("Analysis", "Push left");
        }
        //Drive Forwards to press with lower power, keep pushing for some time
        autonomousDrive.encoderDrive(0.2, 15.0, 15.0, 8.0);
        debug(13);
        autonomousDrive.followLine(0.2, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL, 4.0, false);
        debug(14);
        //Backup, go to other beacon
        autonomousDrive.encoderDrive(0.4, -40.0, -40.0, 5.0);
        debug(15);
        autonomousDrive.pointTurn(0.2, -80.0, 5.0);
        debug(16);
        autonomousDrive.driveTillLine(0.4, 8.0, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL);
        debug(17);
        autonomousDrive.turnTillLine(0.2,AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL,false);
        debug(18);
        autonomousDrive.encoderDrive(0.4,-10.0,-10.0,5.0);
        debug(19);

        String secondBeaconAnalysis = ftcVisionManager.readBeacon(5,10.0);
        debug(20);
        if (beaconAnalysis.equals("redBlue")) {
            //Press Right side b/c we are blue
            telemetry.addData("Analysis", "Push right");
            debug(21);
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            //Turn Servo

        } else {
            //Press Left side b/c we are blue
            debug(22);
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);//Turn Servo
            telemetry.addData("Analysis", "Push left");
        }
        autonomousDrive.encoderDrive(0.2, 15.0, 15.0, 8.0);
        debug(23);
        autonomousDrive.followLine(0.2, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL, 4.0, false);
        debug(24);
        stop();
    }

    public void debug(double i){
        String num = Double.toString(i);
        if(DEBUG){
            telemetry.addData("Debug flag",num);
            return;
        }else{
            return;
        }
    }


}
