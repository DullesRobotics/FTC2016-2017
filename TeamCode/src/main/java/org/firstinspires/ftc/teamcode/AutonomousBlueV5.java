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


@Autonomous(name = "AutonV5_BLUE_right")
public class AutonomousBlueV5 extends LinearVisionOpMode {
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
        autonomousDrive.encoderDrive(.4,170,170,4); //4 seconds = blue
        debug(4);
        autonomousDrive.pointTurn(.4,22.5,2); //Turn right roughly 90
        debug(5);
        servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
        //autonomousDrive.swipeTillLine(.4,3,autonomousDrive.EOPDWHITELINELIGHTLEVEL);
        //autonomousDrive.pointTurn(.4,-5,1);
        //debug(5.1);
        autonomousDrive.encoderDrive(.4,200,200,3); //Forward
        debug(6);
        autonomousDrive.encoderDrive(.2,200,200,2); //Forward slower for beacon
        //autonomousDrive.pointTurn(.4,-5,1);
        debug(7);
        //autonomousDrive.followLine2(6,autonomousDrive.EOPDWHITELINELIGHTLEVEL,4,false);
        //autonomousDrive.followLine(.4,autonomousDrive.EOPDWHITELINELIGHTLEVEL,10,false);
        //debug(8);
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