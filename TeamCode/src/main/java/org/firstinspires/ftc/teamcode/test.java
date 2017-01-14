package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AutonomousDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Kenneth on 1/13/2017.
 */
@Autonomous(name = "adjk")
public class test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotWithFlickerShooter robot;
    AutonomousDrive autonomousDrive;
    OpticalDistanceSensor ods;
    String currentColorOrder = "???, ???";
    int sleepTime = 0;
    ServoControllerLib servoControllerLib;
    final static double ENCODERTICKSPERREVOLUTION = 1478.4;
    final static double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final static double TICKSPERCENTIMETER = ENCODERTICKSPERREVOLUTION/CIRCUMFERENCEOFWHEELCENTIMETERS;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1, hardwareMap.dcMotor.get("flickerShooter"));
        autonomousDrive = new AutonomousDrive(robot,hardwareMap.opticalDistanceSensor.get("EOPD"));
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"),180);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNTOPOSITION();
        telemetry.addData("running", "running");
        robot.getBLM().setTargetPosition((int)(1000));
        robot.getBRM().setTargetPosition((int)(1000));
        robot.getBLM().setPower(.75);
        robot.getBRM().setPower(0.75);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition()<1000){};

        autonomousDrive.resetEncoders();
        autonomousDrive.setRUNWITHENCODERS();
        robot.getBLM().setPower(.5);
        robot.getBRM().setPower(.5);
        while(opModeIsActive()&&robot.getBLM().getCurrentPosition() < 100000&& ods.getLightDetected() < 0.15){

        }
        robot.getBLM().setPower(0.0);
        robot.getBRM().setPower(0.0);
        //autonomousDrive.driveStraightForSetTime(3,0.75);
    }
}
