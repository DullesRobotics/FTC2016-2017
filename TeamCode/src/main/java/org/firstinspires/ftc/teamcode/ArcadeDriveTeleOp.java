package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Kenneth on 11/6/2016.
 * Modified by Karim...heavily
 *
 */

@TeleOp(name = "[MAIN] Arcade Drive")
public class ArcadeDriveTeleOp extends OpMode {
    private AdvancedRobot robot;

    private ArcadeDrive ArcDrive;
    private boolean reversed = true;
    private boolean prevStateReverse = reversed;

    private boolean shooting = true;
    private boolean prevStateShooting = shooting;

    private boolean shootingForwad = false;
    private boolean prevShootingForward = shootingForwad;

    @Override
    public void init() {
        robot = new AdvancedRobot(this);
        ArcDrive = new ArcadeDrive(robot,this);
        robot.setDriveTrain(ArcDrive);
    }

    @Override
    public void loop() {
        boolean curState = robot.getGamepad1().left_bumper;
        boolean curStateY = robot.getGamepad1().right_bumper;
        boolean curStateLB = robot.getGamepad1().y;

        if (curState && (prevStateReverse == false)&& (curState == true)){
            reversed = !reversed;
        }

        if (curStateY && (prevStateShooting == false) && (shooting == true)){
            shooting = !shooting;
        }

        if (curStateLB & (prevShootingForward == false) && (shootingForwad == true)){
            shootingForwad = !shootingForwad;
        }

        prevShootingForward = curStateLB;
        prevStateShooting = curStateY;
        prevStateReverse = curState;

        if (!reversed) {
            robot.drive();
        } else
        {
            robot.reverseDrive();
        }

        if (shooting){
            if (shootingForwad) {
                robot.shoot(1);
            } else {
                robot.shoot(-1);
            }
        } else {
            robot.shoot(0);
        }
    }
}
