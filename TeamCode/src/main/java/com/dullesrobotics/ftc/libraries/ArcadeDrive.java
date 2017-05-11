package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Created by Kenneth on 11/5/2016.
 */

public class ArcadeDrive extends TeleOpDrivetrain{
    OpMode teleop;

    public ArcadeDrive(BasicRobot r, OpMode teleop){
        super(r);
        this.teleop = teleop;
    }

    public ArcadeDrive(AdvancedRobot r, OpMode teleop){
        super(r);
        this.teleop = teleop;
    }

    @Override
    public void driveWithGamepad() {
        Gamepad gamepad = advancedRobot.getGamepad1();
        drive(gamepad.right_stick_x, -gamepad.right_stick_y,-gamepad.left_stick_x);
    }

    public void reverseGamepad(){
        Gamepad gamepad = advancedRobot.getGamepad1();
        drive(-gamepad.right_stick_x, gamepad.right_stick_y,gamepad.left_stick_x);//But I love comments!
    }

    public void shoot(double pow){
        try {
            advancedRobot.getShooterMotor().setPower(pow);
        } catch (NullPointerException e){
            teleop.telemetry.addData("Shooter Motor","Disabled");
            teleop.telemetry.update();
        }
    }

    public void drive(double xPower, double yPower, double strafe) {
        try {
            advancedRobot.getStrafeMotor().setPower(strafe);
        } catch (NullPointerException e) {
            teleop.telemetry.addData("Strafe Motor", "Disabled");
        }
        try {
            advancedRobot.getRightSet().setPower(xPower - yPower);
        } catch (NullPointerException e) {
            teleop.telemetry.addData("Right Motors", "Disabled");
        }
        try {
            advancedRobot.getLeftSet().setPower(xPower + yPower);
        } catch (NullPointerException e) {
            teleop.telemetry.addData("Left Motors", "Disabled");
        }
        teleop.telemetry.update();
    }
}
