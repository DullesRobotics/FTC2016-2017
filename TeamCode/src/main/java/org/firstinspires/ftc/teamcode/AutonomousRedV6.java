package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by nimir on 1/21/2017.
 */


@Autonomous(name = "AutonV6_RED")
public class AutonomousRedV6 extends LinearVisionOpMode {
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
    final boolean bothBeacons = false;
    final boolean parkCorner = false;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();
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
        autonomousDrive.resetAll();
        waitForStart(); //Wait for START Button Press on DS
        debug(3);
        autonomousDrive.encoderDrive(.4,170,170,4.6); //4 seconds = blue
        debug(4);
        autonomousDrive.pointTurn(.4,22.25,2); //Turn right roughly 90
        debug(5);
        autonomousDrive.encoderDrive(.4,200,200,2.1);
        String beaconAnalysis = ftcVisionManager.readBeacon(9,10.0);
        debug(20);
        if (beaconAnalysis.equals("redBlue")) {
            //Press Right side b/c we are blue
            telemetry.addData("Analysis", "Push right");
            debug(21);
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            //Turn Servo

        } else {
            //Press Left side b/c we are blue
            debug(22);
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);//Turn Servo
            telemetry.addData("Analysis", "Push left");
        }
        telemetry.update();
        autonomousDrive.encoderDrive(0.4, 100, 100, 2.05);
        debug(23);
        //autonomousDrive.followLine(0.2, AutonomousDriveClass.EOPDWHITELINELIGHTLEVEL, 4.0, false);
        autonomousDrive.encoderDrive(.4,-200,-200,2.2); //Back up
        if (bothBeacons) {
            debug(24);
            autonomousDrive.pointTurn(.4, -46, 4); //Turn left
            debug(25);
            autonomousDrive.encoderDrive(.4, 100, 100, 3); //Forward
            debug(26);
            autonomousDrive.pointTurn(.4, 22.25, 2.1); //Right
            debug(27);
            autonomousDrive.encoderDrive(.4, 200, 200, 2.1); //Forward
            debug(28);
            beaconAnalysis = ftcVisionManager.readBeacon(5, 10.0);
            debug(20);
            if (beaconAnalysis.equals("redBlue")) {
                //Press Right side b/c we are blue
                telemetry.addData("Analysis", "Push right");
                debug(21);
                servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
                //Turn Servo

            } else {
                //Press Left side b/c we are blue
                debug(22);
                servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);//Turn Servo
                telemetry.addData("Analysis", "Push left");
            }
            telemetry.update();
            autonomousDrive.encoderDrive(0.4, 100, 100, 2.05); // Push slowly
            if (parkCorner){
                debug(22.1);
                autonomousDrive.encoderDrive(.4,-100,-100,2.05); //Back up
                debug(22.2);
                autonomousDrive.pointTurn(.4,30,2.5); //Turn to face corner
                debug(22.3);
                autonomousDrive.encoderDrive(1,100,100,5); //Ram corner
                debug(22.4);
            }
        } else {
            if (parkCorner) {
                debug(22.5);
                autonomousDrive.pointTurn(.4, 30, 2.5); //Turn right to face corner
                debug(22.6);
                autonomousDrive.encoderDrive(1,100,100,5); //Ram corner
                debug(22.7);
            }
        }
        //servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        //autonomousDrive.swipeTillLine(.4,3,autonomousDrive.EOPDWHITELINELIGHTLEVEL);
        //autonomousDrive.pointTurn(.4,-5,1);
        //debug(5.1);
        /*autonomousDrive.encoderDrive(.4,200,200,3); //Forward
        debug(6);
        autonomousDrive.encoderDrive(.2,200,200,2); //Forward slower for beacon*/
    }

    public void debug(double i){
        String num = Double.toString(i);
        if(DEBUG){
            telemetry.addData("Debug flag",num);
            telemetry.update();
            return;
        }else{
            return;
        }
    }
}
