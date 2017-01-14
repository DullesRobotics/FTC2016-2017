package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Kenneth on 1/7/2017.
 */
@Autonomous(name = "SimpleAuton - FAILSAFE")
public class SimpleAuton extends LinearOpMode {
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;
    private ElapsedTime runtime = new ElapsedTime();
    long startTime;
    @Override
    public void runOpMode() throws InterruptedException {
//Start Timer
        startTime = System.currentTimeMillis();





        //Initialize Variables
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        //Set to Arcade Drive (Autonomous Arcade Drive)
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);


        delay(15000);
        if(robot.getFLM() != null) robot.getFLM().setPower(0.5);
        if(robot.getFRM() != null) robot.getFRM().setPower(-0.5);
        if (robot.getBLM() != null) robot.getBLM().setPower(0.5);
        if (robot.getBRM() != null) robot.getBRM().setPower(-0.5);

        delay(3000);

        if(robot.getFLM() != null) robot.getFLM().setPower(0);
        if(robot.getFRM() != null) robot.getFRM().setPower(0);
        if (robot.getBLM() != null) robot.getBLM().setPower(0);
        if (robot.getBRM() != null) robot.getBRM().setPower(0);



    }
    public void delay(long millis){
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start+millis){}
    }
}
