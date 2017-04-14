package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Kenneth on 11/6/2016.
 * Modified by Karim
 *
 */

@TeleOp(name = "[MAIN] Arcade Drive")
public class ArcadeDriveTeleOp extends OpMode {
    private AdvancedRobot robot;

    private ArcadeDrive ArcDrive;
    private boolean reversed = true;
    private boolean prevStateReverse = reversed;

    @Override
    public void init() {
        robot = new AdvancedRobot(
                hardwareMap.dcMotor.get("rightMotors"),
                hardwareMap.dcMotor.get("leftMotors"),
                hardwareMap.dcMotor.get("strafeMotor"),
                gamepad1
        );

        ArcDrive = new ArcadeDrive(robot, this);
        robot.setDriveTrain(ArcDrive);
    }

    @Override
    public void loop() {
        boolean curState = robot.getGamepad1().right_bumper;
        if (curState && (prevStateReverse == false)&&curState == true){
            reversed = !reversed;
        }
        prevStateReverse = curState;
        if (!reversed) {
            robot.drive();
        } else
        {
            robot.reverseDrive();
        }
    }
}
