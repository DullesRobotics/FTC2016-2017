package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.FlickerShooterClass;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.RobotWithWheeledShooter;
import com.dullesrobotics.ftc.libraries.ServoControllerLib;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.dullesrobotics.ftc.libraries.commonMethods.delay;

/**
 * Created by Kenneth on 11/6/2016.
 */

@TeleOp(name = "[MAIN] Arcade Drive")
public class ArcadeDriveTeleOp extends OpMode {
    private AdvancedRobot robot;

    private ArcadeDrive ArcDrive;
    private boolean reversed = true;
    private boolean prevStateReverse = reversed;

    @Override
    public void init() {
        DcMotor[] motors = {
                hardwareMap.dcMotor.get("frontRight"),
                hardwareMap.dcMotor.get("frontLeft"),
                hardwareMap.dcMotor.get("backRight"),
                hardwareMap.dcMotor.get("backLeft"),
                hardwareMap.dcMotor.get("strifeMotor"),
                hardwareMap.dcMotor.get("intakeMotor"),
        };
        robot = new AdvancedRobot(motors, gamepad1);
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);
    }

    @Override
    public void loop() {
        boolean curState = robot.getGamepad().right_bumper;
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
