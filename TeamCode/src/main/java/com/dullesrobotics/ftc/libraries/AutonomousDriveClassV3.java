package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/14/2017.
 *
 * HOW TO USE:
 * -Create an AdvancedRobot object with parameters "this" inside a class
 * that extends LinearVisionOpMode
 *
 * -Create an AutonomousDriveClassV3 object and pass the previously created
 * AdvancedRobot object.
 *
 * Thats it!
 *
 *
 */

public class AutonomousDriveClassV3 {
    private String lightSensorName = "EOPD";
    private String servoName = "buttonServo";

    private double EOPDWhiteLineLightLevel = 0.125;

    private Beacon.AnalysisMethod analysisMethod = Beacon.AnalysisMethod.FAST;
    private int servoInitPosition = ServoControllerLib.SERVORIGHT;


    /**
     *
     *                    YOU DO NOT NEED TO EDIT BELOW
     *
     *
     *        Unless you're adding methods or modifying current methods. Try and change above
     *        without modifying below to meet your needs to keep the code simple for later use.
     *  TODO: Unless you're at a contest, in which case, DO WHATEVER.
     *
     *  --Karim Karim
     *
     * */
    private LinearVisionOpMode opMode;
    private AdvancedRobot robot;
    private FTCVisionManager ftcVisionManager;
    private ServoControllerLib servoControllerLib;
    private LightSensor lightSensor;
    private ElapsedTime timer;

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
    }

    public AutonomousDriveClassV3(AdvancedRobot robot) throws InterruptedException{
        this.robot = robot;
        this.opMode = robot.getLinearVisionOpMode();
        ftcVisionManager = new FTCVisionManager(opMode,analysisMethod);
        try {
            servoControllerLib = new ServoControllerLib(opMode.hardwareMap.servo.get(servoName), servoInitPosition);
        } catch (IllegalArgumentException e){
            opMode.telemetry.addData("ERROR","Unable to find Button Pusher!");
            opMode.telemetry.addData("ERROR CODE",e);
        }
        try {
            lightSensor = opMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        } catch (IllegalArgumentException e){
            opMode.telemetry.addData("ERROR","Unable to find Light Sensor!");
            opMode.telemetry.addData("ERROR CODE",e);
        }
        opMode.telemetry.update();
        timer = new ElapsedTime();
    }

    public void driveSetTime(double time, Direction direction) throws InterruptedException{
        timer.reset();
        int right = 0,left = 0,strafe = 0;
        switch (direction){
            case FORWARD:
                right = 1;
                left = -1;
                break;
            case BACKWARD:
                right = -1;
                left = 1;
                break;
            case RIGHT:
                strafe = 1;
                break;
            case LEFT:
                strafe = -1;
                break;
            default:
                right = 1;
                left = -1;
                break;
        }
        while (opMode.opModeIsActive() && timer.seconds() < time) {
            robot.getRightSet().setPower(right);
            robot.getLeftSet().setPower(left);
            robot.getStrifeMotor().setPower(strafe);
            opMode.telemetry.addData("Autonomous ","Moving " + direction);
            opMode.telemetry.update();
        }
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
        robot.getStrifeMotor().setPower(0);
        opMode.telemetry.addData("Autonomous","Done");
        opMode.telemetry.update();
    }

    public void arcMoveForTime(Direction direction1,Direction direction 2, double time){
        //Im gonna let Kenneth do this, I dont do circles very well

    }
    public void ultimateMove(double xDist, double yDist, double turn) throws Exception {
        throw new Exception("Not implemented yet: Get rekt m8");

    }


}
