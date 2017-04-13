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

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by kk200 on 2/4/2017.
 */

@Disabled
@Autonomous(name = "Full Speed Ahead")
public class Full_Ahead extends LinearVisionOpMode{
    RobotWithFlickerShooter robot;
    AutonomousDriveClassV2 autonomousDrive;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;
    final boolean DEBUG = true;
    ServoControllerLib leftShooter,rightShooter;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();
        debug(1);
        //Initialize Robot
        ftcVisionManager = new FTCVisionManager(this,Beacon.AnalysisMethod.FAST);
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
        //autonomousDrive.encoderDriveInches(.4,57.25,57.25,7); //Forward 1 ft
        autonomousDrive.runForSetTime(1,10);
    }



    /*public void returnToWall() throws InterruptedException{
        autonomousDrive.pointTurn();
    }*/

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
