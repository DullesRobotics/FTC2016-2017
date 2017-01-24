package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDriveClass;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV2;
import com.dullesrobotics.ftc.libraries.FTCVisionManager;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import static com.dullesrobotics.ftc.libraries.commonMethods.*;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by Kenneth on 1/22/2017.
 */
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

        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
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
        logAction("Begin  + SwingTurn");
        autonomousDrive2.swingTurn(0.5,90.0,5.0);
        logAction("End  + Swing Turn");

        delay(2000);

        logAction("Begin  - SwingTurn");
        autonomousDrive2.swingTurn(0.5,-90.0,5.0);
        logAction("End  - Swing Turn");

        delay(2000);

        logAction("Begin  + Point Turn");
        autonomousDrive2.pointTurn(0.5,90.0,5.0);
        logAction("End  + Point Turn");

        delay(2000);

        logAction("Begin  - Point Turn");
        autonomousDrive2.pointTurn(0.5,-90.0,5.0);
        logAction("End  - Point Turn");

    }
    public void logAction(String s){
        telemetry.addData("Action",s);
    }
}
