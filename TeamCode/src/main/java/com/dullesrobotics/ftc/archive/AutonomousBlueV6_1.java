package com.dullesrobotics.ftc.archive;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by nimir on 1/22/2017.
 */
@Disabled
@Autonomous(name = "AutonomousV6.1 RED")
public class AutonomousBlueV6_1 extends LinearVisionOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
        waitForVisionStart();
        this.resetStartTime();
        debug(1);
        //Initialize Robot
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1);
        autonomousDrive = new AutonomousDriveClassV2(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"),servoControllerLib,ftcVisionManager);
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        leftShooter = new ServoControllerLib(hardwareMap.servo.get("catapultOne"));
        rightShooter = new ServoControllerLib(hardwareMap.servo.get("catapultTwo"));
        leftShooter.setDegrees(ServoControllerLib.SERVOLEFT);
        rightShooter.setDegrees(ServoControllerLib.SERVORIGHT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        autonomousDrive.resetEncoders();
        debug(2);
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonomousDrive.resetAll();
        waitForStart(); //Wait for START Button Press on DS
        debug(3);
        autonomousDrive.runForSetTime(0.4,3.0);
        //autonomousDrive.
        //
        String result = ftcVisionManager.readBeacon(7,10);
        if(result.equals("redBlue")){  
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            telemetry.addData("Reader","BLUE_RIGHT");
            telemetry.update();
        }else{
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            telemetry.addData("Reader","BLUE_LEFT");
            if(result.equals("???, ???")){
                telemetry.addData("Reader","Couldn't determine - Defaulting to Left");
            };
            telemetry.update();
        }
        debug(8);
        autonomousDrive.encoderDriveInches(0.3,14,14,5);
        debug(9);
        autonomousDrive.encoderDriveInches(.4,-10,-10,5);
        debug(10);
        autonomousDrive.pointTurn(.4,90.0,2);
        debug(11);
        //autonomousDrive.driveTillLine(0.4,5.0,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL);
        debug(12);
        autonomousDrive.encoderDriveInches(0.4,7.0,7.0,5.0);
        //autonomousDrive.turnTillLine(0.4,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL,false);//Should be facing beacon
        autonomousDrive.pointTurn(0.4,-45.0,3.0);
        result = ftcVisionManager.readBeacon(7,10);
        if(result.equals("redBlue")){
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            telemetry.addData("Reader","BLUE_RIGHT");
            telemetry.update();
        }else{
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            telemetry.addData("Reader","BLUE_LEFT");
            if(result.equals("???, ???")){
                telemetry.addData("Reader","Couldn't determine - Defaulting to Left");
            };
            telemetry.update();
        }
        debug(13);
        autonomousDrive.encoderDriveInches(.3,14,14,5);
        autonomousDrive.encoderDriveInches(0.3,-14,-14,5);


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
}
