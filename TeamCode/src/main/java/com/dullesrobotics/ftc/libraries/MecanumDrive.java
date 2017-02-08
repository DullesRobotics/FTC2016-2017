package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk22 2/8/17
 */

public class MecanumDrive extends TeleOpDrivetrain{

    public MecanumDrive(BasicRobot r){
        super(r);
    }

    @Override
    public void driveWithGamepad() {
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.left_stick_x, -gamepad.left_stick_y); //Don't forget the negatives
        strife(gamepad.right_stick_x,gamepad.right_stick_y);
    }

    public void reverseGamepad(){
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.left_stick_x, gamepad.left_stick_y);
        strife(gamepad.right_stick_x,gamepad.right_stick_y);
    }

    public void drive(double xPower, double yPower) {
        //xPower = xPower/3; //FULL POWAA
        //yPower = yPower/1.5; //FULL POWAA
        if(robot.getFLM() != null) robot.getFLM().setPower(xPower + yPower);
        if(robot.getFRM() != null) robot.getFRM().setPower(xPower - yPower);
        if (robot.getBLM() != null) robot.getBLM().setPower(xPower + yPower);
        if (robot.getBRM() != null) robot.getBRM().setPower(xPower - yPower);
    }

    public void strife(double xPower, double yPower){
        if (robot.getFLM() != null) robot.getFLM().setPower(xPower);
        if (robot.getFRM() != null) robot.getFRM().setPower(-xPower);
        if (robot.getBLM() != null) robot.getBLM().setPower(-xPower);
        if (robot.getBRM() != null) robot.getBRM().setPower(xPower);
    }
}
