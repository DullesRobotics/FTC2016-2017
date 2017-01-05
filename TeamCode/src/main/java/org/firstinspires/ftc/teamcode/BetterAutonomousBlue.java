package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Timer;

/**
 * Created by Kenneth on 1/4/2017.
 */
@Autonomous(name="BLUE Better Autonomous")
public class BetterAutonomousBlue extends LinearOpMode{
    final double ENCODERTICKSPERREVOLUTION = 1478.4;
    final double CIRCUMFERENCEOFWHEELCENTIMETERS = 3.1416*9.6;//ModernRobotics has generously not provided us with a CAD file for the wheel so let's use 96mm diam until we get an actual measurement
    long startTime;
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        //Start Timer
        startTime = System.currentTimeMillis();

        //Initialize Variables
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        //Set to Arcade Drive (Autonomous Arcade Drive)
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);

        //Shoot ONE Ball
        robot.turnBackwards();
        wait(5000);
        robot.stopShooter();

        //Turn ~45deg RIGHT (BLUE TURNS RIGHT TO BEACOM)
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopAndResetEncoders();
        robot.getBLM().setTargetPosition(distToEncoderTicks(2*3.1415*12*2.54/8));//Assuming Robot's turning radius is 12in b/c dist between wheels is ~12in & 45deg is 360/8

        //Keep Going forwards till EOPD detects line
    }
    public double secsSinceStart(){
       return ((Number)(System.currentTimeMillis()-startTime)).doubleValue()/(1000.0);
    }
    public void stopAndResetEncoders() throws InterruptedException {
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wait(1);//Example I used waited one hardware cycle but the method is deprecated
    }
    public int distToEncoderTicks(double dist){
        return (int)(dist/CIRCUMFERENCEOFWHEELCENTIMETERS*ENCODERTICKSPERREVOLUTION);
    }
}
