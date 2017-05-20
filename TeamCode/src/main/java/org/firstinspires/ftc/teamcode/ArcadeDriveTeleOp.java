package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Kenneth on 11/6/2016.
 * Modified by Karim...heavily
 *
 */

@TeleOp(name = "[MAIN] Arcade Drive")
public class ArcadeDriveTeleOp extends OpMode {
    private String btnServoName = "buttonServo";

    private AdvancedRobot robot;
    private ArcadeDrive ArcDrive;
    private ServoControllerLib servoLib;

    private boolean reversed = true;
    private boolean prevStateReverse = reversed;

    @Override
    public void init() {
        robot = new AdvancedRobot(this);
        ArcDrive = new ArcadeDrive(robot,this);
        robot.setDriveTrain(ArcDrive);
        servoLib = new ServoControllerLib(hardwareMap.servo.get(btnServoName),90);
        robot.getRightSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getLeftSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getStrafeMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        boolean curState = robot.getGamepad1().left_bumper;
        double curStateShooter = robot.getGamepad1().right_trigger;

        if (curStateShooter > .1){
            robot.getShooterMotor().setPower(curStateShooter);
        } else {
            robot.getShooterMotor().setPower(0);
        }

        if (curState && (prevStateReverse == false)&& (curState == true)){
            reversed = !reversed;
        }

        prevStateReverse = curState;

        if (!reversed) {
            robot.drive();
        } else
        {
            robot.reverseDrive();
        }

        if (robot.getGamepad1().dpad_right){
            servoLib.setDegrees(ServoControllerLib.SERVORIGHT);
        }

        if (robot.getGamepad1().dpad_left){
            servoLib.setDegrees(ServoControllerLib.SERVOLEFT);
        }
    }
}
