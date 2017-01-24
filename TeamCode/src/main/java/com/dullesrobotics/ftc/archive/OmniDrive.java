package com.dullesrobotics.ftc.archive;

import com.dullesrobotics.ftc.libraries.BasicRobot;
import com.dullesrobotics.ftc.libraries.TeleOpDrivetrain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 11/5/2016.
 */

/*
INFO


OMNIDRIVE ---->> http://ftckey.com/build/drive-trains/

 */

public class OmniDrive extends TeleOpDrivetrain {
    public OmniDrive(BasicRobot r) {
        super(r);
    }



    @Override
    public void driveWithGamepad() {
        Gamepad g = robot.getGamepad1();
        double x = g.left_stick_x;  //X axis translation
        double y = g.left_stick_y;  //Y axis translation
        double c = g.right_stick_x; //Yaw - Rotate about Z axis

        robot.getFLM().setPower(x+y+c);
        robot.getFRM().setPower(-x+y-c);
        robot.getBLM().setPower(-x+y+c);
        robot.getBRM().setPower(x+y-c);
    }

    @Override
    public void reverseGamepad() {
        //xdfbklnbkxf
    }

    @Override
    public void driveQuicklyWithGamepad() {

    }

    @Override
    public void reverseQuicklyGamepad() {

    }
}