package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Kenneth on 1/22/2017.
 */


/**
 * Use this OpMode to measure how far 10000 encoder ticks are
 * Calculate TICKSPERCENTIMETER
 */
@Disabled
@Autonomous(name = "EncoderDistanceMeasure")
public class EncoderDistanceMeasure extends LinearOpMode {
    RobotWithFlickerShooter robot;
    ServoControllerLib servoControllerLib;
    OpticalDistanceSensor ods;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"), hardwareMap.dcMotor.get("BRM"), gamepad1);
        servoControllerLib = new ServoControllerLib(hardwareMap.servo.get("btnServo"), ServoControllerLib.SERVOLEFT);
        robot.getBLM().setDirection(DcMotorSimple.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("EOPD");
        servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getBLM().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getBRM().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getBLM().setTargetPosition(5000);
        robot.getBRM().setTargetPosition(5000); //54 inhces

        telemetry.addData("Starting Left Pos",robot.getBLM().getCurrentPosition());
        telemetry.addData("Starting Right Pos",robot.getBRM().getCurrentPosition());

        robot.getBLM().setPower(0.4);
        robot.getBRM().setPower(0.4);

        while(robot.getBLM().isBusy() || robot.getBRM().isBusy()){
            telemetry.addData("Left Pos",robot.getBLM().getCurrentPosition());
            telemetry.addData("Right Pos",robot.getBRM().getCurrentPosition());
        }

        telemetry.addData("Ending Left Pos",robot.getBLM().getCurrentPosition());
        telemetry.addData("Ending Right Pos",robot.getBRM().getCurrentPosition());
    }
}
