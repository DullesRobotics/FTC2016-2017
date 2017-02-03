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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by nimir on 1/22/2017.
 *
 * EXPLANATION:
 * This OpMode should make the robot:
 * -Init Servos
 * -Wait For Start Button
 * -Wait an additional one second
 * -Fire Catapult One
 * -Wait one second
 * -Fire Catapult Two
 * -Wait one second
 * -Go forward 1 foot
 * -Turn Right
 * -Go forward 6 inches
 * -Turn Left
 * (It should now be facing forward, but infront of the corner vortex, this will make it less prone to hitting the cap-ball)
 * -Go forward 3 feet (Robot should now be in front of beacon, but not facing it)
 * -Turn right (Facing beacon now)
 * -Go forward 1 foot (to get closer to beacon)
 * --TODO: Attempt to add FollowLine here
 * -Begins reading beacon and attempts to push it till the beacon is the correct color
 * --End of OpMode
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

    final static double CATAPULT_ONE_INIT = 0; //Degrees to move to during Init
    final static double CATAPULT_TWO_INIT = 0;

    final static double CATAPULT_ONE_FIRE = 0; //Degrees to move to to fire catapault
    final static double CATAPULT_TWO_FIRE = 0;

    private Servo[] servos = {hardwareMap.servo.get("btnServo"), hardwareMap.servo.get("catapaultOne"),hardwareMap.servo.get("catapaultTwo")};

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
        servoControllerLib = new ServoControllerLib(servos);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        autonomousDrive.resetEncoders();
        debug(2);
        autonomousDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonomousDrive.resetAll();
        servoControllerLib.setDegrees(CATAPULT_ONE_INIT,1);
        servoControllerLib.setDegrees(CATAPULT_TWO_INIT,2);
        waitForStart(); //Wait for START Button Press on DS
        delay(1000);
        autonomousDrive.fireCatapult(CATAPULT_ONE_FIRE,1);
        delay(1000);
        autonomousDrive.fireCatapult(CATAPULT_TWO_FIRE,2);
        delay(1000);
        debug(3);
        autonomousDrive.encoderDriveInches(.4,12,12,2.5); //Forward 1 ft
        debug(4);
        autonomousDrive.pointTurn(.4,45,3); //Turn right
        debug(5);
        autonomousDrive.encoderDriveInches(.4,6,6,1); //Go forward 6 inches
        debug(6);
        autonomousDrive.pointTurn(.4,-45,2); //Turn left
        debug(7);
        autonomousDrive.encoderDriveInches(.4,24,24,4.5); //Forward 3 feet (robot should be in front of beacon now, not facing)
        debug(8);
        autonomousDrive.pointTurn(.4,90,2); //Turn roughly 90 to face beacon
        debug(9);
        autonomousDrive.encoderDriveInches(.4,12,12,1); //Go forward around 1 foot to get closer to beacon
        debug(10);
        String beaconAnalysisOne = ftcVisionManager.readBeacon(9,10); //redBlue = blue on left, blueRed = blue on right
        autonomousDrive.readAndPush(beaconAnalysisOne,3,"blue"); //Theoretically should push beacon till it changes (3 = max tries)
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