package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 2/17/2017.
 *
 * TODO: This seems to not turn correctly, although it should be fixed.
 * TODO: We need to test.
 */

@Autonomous(name = "Autonomous Test Red")
public class TestClassRed extends LinearVisionOpMode {
    ElapsedTime runtime = new ElapsedTime();
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    RobotWithFlickerShooter robot;
    AutonomousDriveClassV2 autonomousDrive;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;
    final boolean DEBUG = true;
    final boolean bothBeacons = false;
    final boolean parkCorner = false;
    ServoControllerLib leftShooter,rightShooter;

    @Override
    public void runOpMode() throws InterruptedException {
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1);
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        autonomousDrive = new AutonomousDriveClassV2(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"),servoControllerLib,ftcVisionManager);
        //leftShooter = new ServoControllerLib(hardwareMap.servo.get("catapultOne"));
        //rightShooter = new ServoControllerLib(hardwareMap.servo.get("catapultTwo"));
        //leftShooter.setDegrees(ServoControllerLib.SERVOLEFT);
        //rightShooter.setDegrees(ServoControllerLib.SERVORIGHT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        autonomousDrive.resetEncoders();
        debug(2);
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonomousDrive.resetAll();
        waitForStart(); //Wait for START Button Press on DS
        debug(3);
        autonomousDrive.setEOPDWHITELINELIGHTLEVEL(1.5,20);
        debug("Moving forward 2 feet");
        autonomousDrive.encoderDriveInches(.4,24,3);
        debug("Turning 55 degrees");
        autonomousDrive.pointTurn(.4,-50,3);
        debug("Driving till line");
        autonomousDrive.driveTillLine(.4,10);
        autonomousDrive.pointTurn(.2,-2.5,.35);
        //If robot hits wall, then make robot back-up first
        debug("Turning 30 degrees to line up better with line");
        autonomousDrive.pointTurn(.4,-40,3);
        debug("Attempting to follow line");
        autonomousDrive.followLine(.2,5,4);
        debug("Robot should be facing beacon. Beginning analysis");
        String analysis = ftcVisionManager.readBeacon(7,4);
        autonomousDrive.readAndSetServo("blue");
        debug("Pressing beacon");
        autonomousDrive.encoderDriveInches(.3,-18,2.5);
        autonomousDrive.encoderDriveInches(.4,5,2.5);
        //Robot should have pressed first beacon by now
        /*debug("Moving onto second beacon");
        autonomousDrive.encoderDriveInches(.4,36,4);
        debug("Heading forward one foot");
        autonomousDrive.encoderDriveInches(.4,12,3); //If robot goes forward twice, fix this
        debug("Turning to aim for white line");
        autonomousDrive.pointTurn(.4,55,3);
        debug("Driving till line 2");
        autonomousDrive.driveTillLine(.4,10);
        autonomousDrive.pointTurn(.2,2.4,.35);
        debug("Turning to line up better");
        autonomousDrive.pointTurn(.4,30,3);
        debug("Attempting to follow line");
        autonomousDrive.followLine(.2,5,4);
        debug("Robot should be facing beacon. Beginning analysis");
        analysis = ftcVisionManager.readBeacon(7,4);
        autonomousDrive.readAndSetServo("blue");
        debug("Pressing beacon");
        autonomousDrive.encoderDriveInches(.3,6,2.5);*/
    }
    public void debug(double i) throws InterruptedException{
        String num = Double.toString(i);
        if(DEBUG){
            telemetry.addData("Debug flag",num);
            telemetry.update();
            return;
        }else{
            return;
        }
    }

    public void debug(String data) throws InterruptedException{
        if(DEBUG){
            telemetry.addData("Debug flag 2",data);
            telemetry.update();

            return;
        }else{
            return;
        }
    }
}
