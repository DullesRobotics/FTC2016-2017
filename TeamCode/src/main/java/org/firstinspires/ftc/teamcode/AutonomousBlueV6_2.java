package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
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

import static android.R.attr.delay;

/**
 * Created by kk200 on 2/4/2017.
 */

@Autonomous(name = "AutonomousV6.2 BLUE")
public class AutonomousBlueV6_2 extends LinearVisionOpMode{
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
        //autonomousDrive.encoderDriveInches(.4,57.25,57.25,7); //Forward 1 ft
        autonomousDrive.encoderDriveInches(.4,18,18,3);
        debug(4);
        autonomousDrive.pointTurn(0.4,183.0,2.0);//TURN 180
        /* Shoot Balls */
        leftShooter.setDegrees(ServoControllerLib.SERVORIGHT);
        rightShooter.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive.pointTurn(0.4,195.0,2.5);
        debug(5);
        autonomousDrive.encoderDriveInches(.4,40,40,4);
        debug(6);
        autonomousDrive.pointTurn(.4,115,2);
        debug(7);
        autonomousDrive.encoderDriveInches(.4,24,24,5);
        debug(8);
        String result = ftcVisionManager.readBeacon(7,4);
        if (result.equals("redBlue")){
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        } else {
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        }
        debug(9);
        autonomousDrive.encoderDriveInches(.25,10,10,5);
        debug(10);
        autonomousDrive.encoderDriveInches(.4,-21,-21,5);
        debug(11);
        autonomousDrive.pointTurn(.4,-105,2);
        debug(12);
        autonomousDrive.encoderDriveInches(.4,45,45,4);
        debug(13);
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
