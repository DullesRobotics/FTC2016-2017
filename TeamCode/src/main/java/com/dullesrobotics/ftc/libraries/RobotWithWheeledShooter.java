package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 12/27/2016.
 */
public class RobotWithWheeledShooter extends BasicRobot {
    private DcMotor shooterMotor;

    public RobotWithWheeledShooter(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2) {
        super(frontLeft, frontRight, backLeft, backRight, g1, g2);
    }
}
