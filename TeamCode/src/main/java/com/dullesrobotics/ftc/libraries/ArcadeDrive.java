package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 11/5/2016.
 */

public class ArcadeDrive extends TeleOpDrivetrain{
    OpMode teleop;

    public ArcadeDrive(BasicRobot r){
        super(r);
    }

    public ArcadeDrive(AdvancedRobot r){ super(r); }

    public ArcadeDrive(AdvancedRobot r, OpMode teleop){ super(r); this.teleop = teleop; }

    @Override
    public void driveWithGamepad() {
        Gamepad gamepad = advancedRobot.getGamepad1();
        drive(-gamepad.right_stick_x, -gamepad.right_stick_y,gamepad.left_stick_x);
    }

    public void reverseGamepad(){
        Gamepad gamepad = advancedRobot.getGamepad1();
        drive(gamepad.right_stick_x, gamepad.right_stick_y,-gamepad.left_stick_x);
    }

    public void drive(double xPower, double yPower, double strafe) {
        /*if(advancedRobot.getFrontLeft() != null) advancedRobot.getFrontLeft().setPower(xPower + yPower);
        if(advancedRobot.getFrontRight() != null) advancedRobot.getFrontRight().setPower(xPower - yPower);
        if (advancedRobot.getBackLeft() != null) advancedRobot.getBackLeft().setPower(xPower + yPower);
        if (advancedRobot.getBackRight() != null) advancedRobot.getBackRight().setPower(xPower - yPower);*/
        if (advancedRobot.getStrifeMotor() != null) {
            advancedRobot.getStrifeMotor().setPower(strafe);
            if (teleop != null) {
                teleop.telemetry.addData("Strafe Motor", "Enabled");
                teleop.telemetry.addData("Strafe Motor Power", advancedRobot.getStrifeMotor().getPower());
            }
        } else {
            if (teleop != null) {
                teleop.telemetry.addData("Strafe Motor", "Disabled");
            }
        }
        if (advancedRobot.getRightSet() != null) {
            advancedRobot.getRightSet().setPower(xPower - yPower);
            if (teleop != null) {
                teleop.telemetry.addData("Right Motors", "Enabled");
                teleop.telemetry.addData("Right Motor Power", advancedRobot.getRightSet().getPower());
            }
        } else {
            if (teleop != null) {
                teleop.telemetry.addData("Right Motors", "Disabled");
            }
        }
        if (advancedRobot.getLeftSet() != null) {
            advancedRobot.getLeftSet().setPower(xPower + yPower);
            if (teleop != null) {
                teleop.telemetry.addData("Left Motors", "Enabled");
                teleop.telemetry.addData("Left Motor Power", advancedRobot.getLeftSet().getPower());
            }
        } else {
            if (teleop != null) {
                teleop.telemetry.addData("Left Motors", "Disabled");
            }
        }
        if (teleop != null) {
            teleop.telemetry.update();
        }
    }
}
