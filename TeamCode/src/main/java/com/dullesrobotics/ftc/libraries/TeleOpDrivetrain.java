package com.dullesrobotics.ftc.libraries;

/**
 * Created by Kenneth on 11/5/2016.
 */

public abstract class TeleOpDrivetrain {

    protected BasicRobot robot;
    protected AdvancedRobot robot2;

    public TeleOpDrivetrain(BasicRobot r){ robot = r; }

    public TeleOpDrivetrain(AdvancedRobot r){ robot2 = r; }

    public abstract void driveWithGamepad();
    public abstract void reverseGamepad();
}
