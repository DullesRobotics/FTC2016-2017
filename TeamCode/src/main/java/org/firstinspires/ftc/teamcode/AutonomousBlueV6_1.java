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

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.resetStartTime();
        debug(1);
        //Initialize Robot
        ftcVisionManager = new FTCVisionManager(this, Beacon.AnalysisMethod.FAST);
        ftcVisionManager.initFTCVision();
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDriveClassV2(this, robot, hardwareMap.opticalDistanceSensor.get("EOPD"),servoControllerLib,ftcVisionManager);
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive.resetEncoders();
        debug(2);
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonomousDrive.resetAll();
        waitForStart(); //Wait for START Button Press on DS
        debug(3);
        autonomousDrive.encoderDriveInches(.4,12,12,2.5);
        debug(4);
        autonomousDrive.pointTurn(.4,45,3); //Turn right
        debug(5);
        autonomousDrive.encoderDriveInches(.4,6,6,1); //Go forward 6 inches
        debug(6);
        autonomousDrive.pointTurn(.4,-45,2); //Turn left
        debug(7);
        autonomousDrive.encoderDriveInches(.4,24,24,4.5); //Forward 3 feet (robot should be in front of beacon now)
        debug(8);
        autonomousDrive.pointTurn(.4,90,2); //Turn roughly 90 to face beacon
        debug(9);
        autonomousDrive.encoderDriveInches(.4,12,12,1); //Go forward around 1 foot to get closer to beacon
        debug(10);
        String beaconAnalysisOne = ftcVisionManager.readBeacon(9,10); //redBlue = blue on left, blueRed = blue on right
        autonomousDrive.readAndPush(beaconAnalysisOne,3); //Theoretically should push beacon till it changes (3 = max tries)
        debug(11);
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
