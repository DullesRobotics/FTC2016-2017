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
        Gamepad gamepad = basicRobot.getGamepad1();
        drive(-gamepad.left_stick_x, -gamepad.left_stick_y);
        strafe(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public void reverseGamepad() {
        Gamepad gamepad = basicRobot.getGamepad1();
        drive(-gamepad.left_stick_x, gamepad.left_stick_y);
        strafe(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    public void drive(double xPower, double yPower) {
        //xPower = xPower/3; //FULL POWAA
        //yPower = yPower/1.5; //FULL POWAA
        if (basicRobot.getFLM() != null) basicRobot.getFLM().setPower(xPower + yPower);
        if (basicRobot.getFRM() != null) basicRobot.getFRM().setPower(xPower - yPower);
        if (basicRobot.getBLM() != null) basicRobot.getBLM().setPower(xPower + yPower);
        if (basicRobot.getBRM() != null) basicRobot.getBRM().setPower(xPower - yPower);
    }

    public void strafe(double xPower, double yPower) {
        DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(yPower > 0 && xPower>0 && yPower > xPower){
            if (basicRobot.getFRM() != null) basicRobot.getFRM().setZeroPowerBehavior(behavior);
            if (basicRobot.getBLM() != null) basicRobot.getBLM().setZeroPowerBehavior(behavior);
            if (basicRobot.getFLM() != null) basicRobot.getFLM().setPower(yPower + xPower);
            if (basicRobot.getBRM() != null) basicRobot.getBRM().setPower(yPower + xPower);

        }
        else if(yPower > 0 && xPower<0 && yPower > xPower){
            if (basicRobot.getFLM() != null) basicRobot.getFLM().setZeroPowerBehavior(behavior);
            if (basicRobot.getBRM() != null) basicRobot.getBRM().setZeroPowerBehavior(behavior);
            if (basicRobot.getFRM() != null) basicRobot.getFLM().setPower(xPower - yPower);
            if (basicRobot.getBLM() != null) basicRobot.getBLM().setPower(xPower - yPower);

        }
        else if(xPower<0 && yPower<0 && yPower > xPower){
            if (basicRobot.getFRM() != null) basicRobot.getFRM().setZeroPowerBehavior(behavior);
            if (basicRobot.getBLM() != null) basicRobot.getBLM().setZeroPowerBehavior(behavior);
            if (basicRobot.getFLM() != null) basicRobot.getFLM().setPower(-yPower - xPower);
            if (basicRobot.getBRM() != null) basicRobot.getBRM().setPower(-yPower - xPower);
        }
        else if(xPower>0 && yPower<0 && yPower > xPower){
            if (basicRobot.getFLM() != null) basicRobot.getFLM().setZeroPowerBehavior(behavior);
            if (basicRobot.getBRM() != null) basicRobot.getBRM().setZeroPowerBehavior(behavior);
            if (basicRobot.getFRM() != null) basicRobot.getFLM().setPower(-xPower - yPower);
            if (basicRobot.getBLM() != null) basicRobot.getBLM().setPower(-xPower - yPower);
        } else {
            if (basicRobot.getFLM() != null) basicRobot.getFLM().setPower(xPower);
            if (basicRobot.getFRM() != null) basicRobot.getFRM().setPower(-xPower);
            if (basicRobot.getBLM() != null) basicRobot.getBLM().setPower(-xPower);
            if (basicRobot.getBRM() != null) basicRobot.getBRM().setPower(xPower);
        }

    }

}