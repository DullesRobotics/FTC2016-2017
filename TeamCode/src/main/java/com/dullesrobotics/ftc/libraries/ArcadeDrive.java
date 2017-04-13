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
        Gamepad gamepad = advancedRobot.getGamepad();
        drive(-gamepad.right_stick_x, -gamepad.right_stick_y,-gamepad.left_stick_x, gamepad.left_trigger); //Don't forget the negatives
    }

    public void reverseGamepad(){
        Gamepad gamepad = advancedRobot.getGamepad();
        drive(gamepad.right_stick_x, gamepad.right_stick_y,gamepad.left_stick_x,gamepad.left_trigger);
    }

    public void drive(double xPower, double yPower, double strafe, double intake) {
        //xPower = (xPower/1.25);
        yPower = (yPower/2);
        if(advancedRobot.getFrontLeft() != null) advancedRobot.getFrontLeft().setPower(xPower + yPower);
        if(advancedRobot.getFrontRight() != null) advancedRobot.getFrontRight().setPower(xPower - yPower);
        if (advancedRobot.getBackLeft() != null) advancedRobot.getBackLeft().setPower(xPower + yPower);
        if (advancedRobot.getBackRight() != null) advancedRobot.getBackRight().setPower(xPower - yPower);
        if (advancedRobot.getStrifeMotor() != null) advancedRobot.getStrifeMotor().setPower(strafe);
        if (advancedRobot.getBallIntake() != null) advancedRobot.getBallIntake().setPower(intake);
    }
}
