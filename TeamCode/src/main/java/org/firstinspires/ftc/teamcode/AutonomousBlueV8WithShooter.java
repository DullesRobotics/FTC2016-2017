package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import com.dullesrobotics.ftc.libraries.AdvancedRobot;
import com.dullesrobotics.ftc.libraries.AutonomousDriveClassV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/13/2017.
 */

@Autonomous(name = "[BLUE] Autonomous V8 With Shooter")

public class AutonomousBlueV8WithShooter extends LinearVisionOpMode {
    AutonomousDriveClassV3 autonomous;
    AdvancedRobot robot;
    DcMotor shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AdvancedRobot(this);
        autonomous = new AutonomousDriveClassV3(robot);
        shooter = robot.getShooterMotor();
        double defaultSpeed = .75;
        //String TeamOn = "Blue";
        robot.getRightSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getLeftSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getStrafeMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        telemetry.update();

        /*
       autonomous.driveTillLine(AutonomousDriveClassV3.Direction.FORWARD,.5,10);
       */

        /*Start out sideways facing corner vortex
        Left for a few seconds
        Forward till sensor can touch line when moved left
        Left till line
        SetServo
        Forward
        Back
        Left for .5 seconds
        Left till line
        SetServo
        Forward
        Back

        TODO: Add strifing before hitting left; not right

        TODO: Calibrate EOPD
         */
        autonomous.driveSetTime(2.3, AutonomousDriveClassV3.Direction.LEFT,0.4);
        autonomous.driveSetTime(2.2, AutonomousDriveClassV3.Direction.FORWARD,0.2);
        autonomous.driveTillLine(AutonomousDriveClassV3.Direction.LEFT,0.2, 5.0);
        autonomous.driveSetTime(0.23101, AutonomousDriveClassV3.Direction.LEFT,0.1501);
        autonomous.setServo("blue");
        autonomous.driveSetTime(2.3, AutonomousDriveClassV3.Direction.FORWARD, 0.2);
        //ADD TODO here
        autonomous.driveSetTime(1.3, AutonomousDriveClassV3.Direction.BACKWARD, 0.25);
        autonomous.driveSetTime(.5, AutonomousDriveClassV3.Direction.LEFT, 0.2);
        autonomous.driveTillLine(AutonomousDriveClassV3.Direction.LEFT, 0.3, 5.0);
        autonomous.setServo("blue");
        autonomous.driveSetTime(2.0, AutonomousDriveClassV3.Direction.FORWARD, 0.3);
        autonomous.driveSetTime(2.0, AutonomousDriveClassV3.Direction.BACKWARD, 0.3);
    }
}
