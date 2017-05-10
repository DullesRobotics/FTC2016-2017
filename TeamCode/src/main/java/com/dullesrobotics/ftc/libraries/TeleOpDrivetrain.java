package com.dullesrobotics.ftc.libraries;

/**
 * Created by Kenneth on 11/5/2016.
 */

public abstract class TeleOpDrivetrain {

    protected BasicRobot basicRobot;
    protected AdvancedRobot advancedRobot;

    public TeleOpDrivetrain(BasicRobot r){ basicRobot = r; }

    public TeleOpDrivetrain(AdvancedRobot r){ advancedRobot = r; }

    public abstract void driveWithGamepad();
    public abstract void reverseGamepad();
    public abstract void shoot(double pow);
}
