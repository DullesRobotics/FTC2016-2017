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

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by kk200 on 2/4/2017.
 */

@Autonomous(name = "AutonomousV6.2 RED")
public class AutonomousRedV6_2 extends LinearVisionOpMode{
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
        autonomousDrive.encoderDriveInches(.3,18,18,3);
        /*debug(4);
        autonomousDrive.pointTurn(0.3,200,2.0);//TURN 180

        leftShooter.setDegrees(ServoControllerLib.SERVORIGHT);
        rightShooter.setDegrees(ServoControllerLib.SERVOLEFT);
        delay(500);
        autonomousDrive.pointTurn(0.4,183,2.5);
        debug(5);
        autonomousDrive.encoderDriveInches(.65,41.5,41.5,3.5);
        debug(6);
        autonomousDrive.pointTurn(.4,-100,2);
        debug(7);
        autonomousDrive.encoderDriveInches(.5,24,24,4);
        debug(8);
        String result = ftcVisionManager.readBeacon(7,4);
        if (result.equals("blueRed")){
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        } else {
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        }
        debug(9);
        autonomousDrive.encoderDriveInches(.25,15,15,5);
        debug(10);
        autonomousDrive.encoderDriveInches(.4,-21,-21,5);
        debug(11);
        autonomousDrive.pointTurn(.4,105,2);
        debug(12);
        autonomousDrive.encoderDriveInches(.4,45,45,4);
        debug(13);
        autonomousDrive.pointTurn(.4,-100,2);
        debug(14);
        autonomousDrive.encoderDriveInches(.4,24,24,4);
        debug(15);
        result = ftcVisionManager.readBeacon(7,4);
        if (result.equals("blueRed")){
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        } else {
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        }
        autonomousDrive.encoderDriveInches(.24,10,10,5);
        debug(16);
        autonomousDrive.encoderDriveInches(.4,-21,-21,3);
        debug(17);
        autonomousDrive.pointTurn(.4,-80,2);
        debug(18);
        autonomousDrive.encoderDriveInches(.7,144,144,3);*/
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