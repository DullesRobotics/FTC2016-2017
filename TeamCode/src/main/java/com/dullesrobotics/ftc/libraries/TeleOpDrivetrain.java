package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 11/5/2016.
 */

public abstract class TeleOpDrivetrain {

    protected BasicRobot robot;

    public TeleOpDrivetrain(BasicRobot r){
        robot = r;
    }

    public abstract void driveWithGamepad();
    public abstract void reverseGamepad();
    //public abstract void driveQuicklyWithGamepad();
    //public abstract void reverseQuicklyGamepad();

}
