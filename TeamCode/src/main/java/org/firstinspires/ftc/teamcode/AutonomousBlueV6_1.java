package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by nimir on 1/22/2017.
 */

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

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();
        debug(1);
        //Initialize Robot
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDriveClassV2(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"));
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
        autonomousDrive.encoderDrive(1,60.96,60.96,2); //Go forward 2 feet
        debug(4);
        autonomousDrive.pointTurn(1,22.25,2); //Turn right
        debug(5);
        autonomousDrive.encoderDrive(1,15.24,15.25,1); //Go forward 6 inches
        debug(6);
        autonomousDrive.pointTurn(1,-22.25,2); //Turn left
        debug(7);
        autonomousDrive.encoderDrive(1,91.44,91.44,4.5); //Forward 3 feet (robot should be in front of beacon now)
        debug(8);
        autonomousDrive.pointTurn(1,22.25,2); //Turn roughly 90 to face beacon
        debug(9);
        autonomousDrive.encoderDrive(1,30,30,1); //Go forward around 1 foot to get closer to beacon
        debug(10);
        String beaconAnalysisOne = ftcVisionManager.readBeacon(9,10); //redBlue = blue on left, blueRed = blue on right
        readAndPush(beaconAnalysisOne); //Theoretically should push beacon till its changed
        debug(23);
    }

    public void readAndPush(String analysis) throws InterruptedException{
        //Assuming the beacon is randomized, which it should be.....
        waitOneFullHardwareCycle();
        if (analysis.equals("redBlue")) { //Blue is on left
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            autonomousDrive.encoderDrive(.4, 100, 100, 5);
            autonomousDrive.encoderDrive(.4, -100, -100, 5);
            servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
            telemetry.addData("read and push status","Pushed left");
            telemetry.update();
        } else {
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            autonomousDrive.encoderDrive(.4, 100, 100, 5);
            autonomousDrive.encoderDrive(.4, -100, -100, 5);
            servoControllerLib.setDegrees(ServoControllerLib.SERVORIGHT);
            telemetry.addData("read and push status","Pushed right");
            telemetry.update();
        }
        String ourAnalysis = ftcVisionManager.readBeacon(9,10);
        if (ourAnalysis.equals(analysis)){
            telemetry.addData("read and push status","Trying again");
            telemetry.update();
            readAndPush(ourAnalysis);
        }
        telemetry.addData("read and push status","Successfully completed!");
        telemetry.update();
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
