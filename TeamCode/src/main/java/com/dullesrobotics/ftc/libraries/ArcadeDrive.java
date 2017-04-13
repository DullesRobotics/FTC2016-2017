package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 11/5/2016.
 */

public class ArcadeDrive extends TeleOpDrivetrain{

    public ArcadeDrive(BasicRobot r){
        super(r);
    }

    public ArcadeDrive(AdvancedRobot r){ super(r); }

    @Override
    public void driveWithGamepad() {
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.right_stick_x, -gamepad.right_stick_y,-gamepad.left_stick_x); //Don't forget the negatives
    }

    public void reverseGamepad(){
        Gamepad gamepad = robot.getGamepad1();
        drive(-gamepad.right_stick_x, gamepad.right_stick_y,gamepad.left_stick_x);
    }

    public void drive(double xPower, double yPower, double strafe) {
        //xPower = (xPower/1.25);
        yPower = (yPower/2);
        if(robot.getFLM() != null) robot.getFLM().setPower(xPower + yPower);
        if(robot.getFRM() != null) robot.getFRM().setPower(xPower - yPower);
        if (robot.getBLM() != null) robot.getBLM().setPower(xPower + yPower);
        if (robot.getBRM() != null) robot.getBRM().setPower(xPower - yPower);
        if (robot.getStrifeMotor() != null) robot.getStrifeMotor().setPower(strafe);

    }
}
