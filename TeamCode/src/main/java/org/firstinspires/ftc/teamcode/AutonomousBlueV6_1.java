package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
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
 * Created by nimir on 1/22/2017.
 */

@Autonomous(name = "AutonomousV6.1 BLUE")
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
        //autonomousDrive.encoderDriveInches(.4,57.25,57.25,7); //Forward 1 ft
        autonomousDrive.encoderDriveInches(.4,18,18,7);
        debug(4);

        autonomousDrive.pointTurn(0.4,183.0,2.0);//TURN 180
        /* Shoot Balls */
        leftShooter.setDegrees(ServoControllerLib.SERVORIGHT);
        rightShooter.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive.pointTurn(0.4,205.0,3.0);

        //Turn 45deg
        autonomousDrive.pointTurn(0.4,30.0,5.0);
        //Drive till line
        autonomousDrive.driveTillLine(0.4,6.0,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL);
        autonomousDrive.encoderDrive(0.4,20,20,5.0);//Go fwd a bit
        autonomousDrive.pointTurn(0.4,25.0,3.0);
        autonomousDrive.encoderDrive(0.4,-5.0,-5.0,2.0);
        autonomousDrive.pointTurn(0.4,30.0,3.0);
        //autonomousDrive.turnTillLine(0.25,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL,false);
        autonomousDrive.encoderDriveInches(0.25,-5,-5,5.0);
        /*
        autonomousDrive.pointTurn(.4,102.5,2); //Turn left
        debug(7);
        autonomousDrive.encoderDriveInches(.4,22,22,2);

        */
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
        autonomousDrive.pointTurn(.4,-90.0,2);
        debug(11);
        autonomousDrive.driveTillLine(0.4,5.0,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL);
        debug(12);
        autonomousDrive.encoderDriveInches(0.4,7.0,7.0,5.0);
        //autonomousDrive.turnTillLine(0.4,AutonomousDriveClassV2.EOPDWHITELINELIGHTLEVEL,false);//Should be facing beacon
        autonomousDrive.pointTurn(0.4,45.0,3.0);
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
