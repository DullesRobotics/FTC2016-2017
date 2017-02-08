package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 2/7/2017.
 */


public class RobotWithMecanumWheels extends RobotWithFlickerShooter {

    public RobotWithMecanumWheels(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2,DcMotor shooter) {
        super(frontLeft, frontRight, backLeft, backRight, g1, g2, shooter);
    }

    //Add move methods
}
