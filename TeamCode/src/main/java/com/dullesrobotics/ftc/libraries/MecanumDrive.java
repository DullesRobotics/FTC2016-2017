package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk22 2/8/17
 */

public class MecanumDrive extends TeleOpDrivetrain {

    public MecanumDrive(BasicRobot r) {
        super(r);
    }

    @Override
    public void driveWithGamepad() {
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.left_stick_x, -gamepad.left_stick_y); //Don't forget the negatives
        strafe(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public void reverseGamepad() {
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.left_stick_x, gamepad.left_stick_y);
        strafe(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public void drive(double xPower, double yPower) {
        //xPower = xPower/3; //FULL POWAA
        //yPower = yPower/1.5; //FULL POWAA
        if (robot.getFLM() != null) robot.getFLM().setPower(xPower + yPower);
        if (robot.getFRM() != null) robot.getFRM().setPower(xPower - yPower);
        if (robot.getBLM() != null) robot.getBLM().setPower(xPower + yPower);
        if (robot.getBRM() != null) robot.getBRM().setPower(xPower - yPower);
    }

    public void strafe(double xPower, double yPower) {
        DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(yPower > 0 && xPower>0){
            robot.getFRM().setZeroPowerBehavior(behavior);
            robot.getBLM().setZeroPowerBehavior(behavior);
            if (robot.getFLM() != null) robot.getFLM().setPower(xPower);
            if (robot.getBRM() != null) robot.getBRM().setPower(xPower);

        }
        else if(yPower > 0 && xPower<0){
            robot.getFLM().setZeroPowerBehavior(behavior);
            robot.getBRM().setZeroPowerBehavior(behavior);
            if (robot.getFRM() != null) robot.getFLM().setPower(yPower);
            if (robot.getBLM() != null) robot.getBLM().setPower(yPower);

        }
        else if(xPower<0 && yPower<0){
            robot.getFRM().setZeroPowerBehavior(behavior);
            robot.getBLM().setZeroPowerBehavior(behavior);
            if (robot.getFLM() != null) robot.getFLM().setPower(xPower);
            if (robot.getBRM() != null) robot.getBRM().setPower(xPower);
        }
        else if(xPower>0 && yPower<0){
            robot.getFLM().setZeroPowerBehavior(behavior);
            robot.getBRM().setZeroPowerBehavior(behavior);
            if (robot.getFLM() != null) robot.getFRM().setPower(yPower);
            if (robot.getBRM() != null) robot.getBLM().setPower(yPower);
        } else {
            if (robot.getFLM() != null) robot.getFLM().setPower(xPower);
            if (robot.getFRM() != null) robot.getFRM().setPower(-xPower);
            if (robot.getBLM() != null) robot.getBLM().setPower(-xPower);
            if (robot.getBRM() != null) robot.getBRM().setPower(xPower);
        }

    }

}
