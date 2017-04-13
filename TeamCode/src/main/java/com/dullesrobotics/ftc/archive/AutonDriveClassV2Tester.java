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

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by Kenneth on 1/22/2017.
 */
@Disabled
@Autonomous(name = "AutonDriveClassV2Tester")
public class AutonDriveClassV2Tester extends LinearVisionOpMode {
    RobotWithFlickerShooter robot;
    AutonomousDriveClassV2 autonomousDrive2;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    FTCVisionManager ftcVisionManager;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();

        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1);
        autonomousDrive2 = new AutonomousDriveClassV2(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"),servoControllerLib,ftcVisionManager);
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive2.resetEncoders();

        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        autonomousDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonomousDrive2.resetAll();
        waitForStart(); //Wait for START Button Press on DS

        /**
         * INSERT CODE TO TEST BELOW THIS COMMENT
         */
        logAction("Begin  driveToLine");
        //autonomousDrive2.driveTillLine(0.4,10.0,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL);


    }
    public void logAction(String s){
        telemetry.addData("Action",s);
    }
}
