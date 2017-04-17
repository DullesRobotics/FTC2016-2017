package com.dullesrobotics.ftc.libraries;

import android.test.InstrumentationTestRunner;

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
 * that extends LinearVisionOpMode (it must extend that op mode)
 *
 * -Create an AutonomousDriveClassV3 object and pass the previously created
 * AdvancedRobot object.
 *
 * Thats it!
 *
 *
 */

public class AutonomousDriveClassV3 {
    private String lightSensorName = "EOPD"; //light sensor's name
    private String servoName = "buttonServo"; //button pusher servo's name

    private double WhiteLineLightLevel = 0.125; //White line light level

    private Beacon.AnalysisMethod analysisMethod = Beacon.AnalysisMethod.FAST; //Beacon analysis method...self-explanatory
    private int servoInitPosition = ServoControllerLib.SERVORIGHT; //Initial servo position

    private boolean delayAfterEachCall = true; //Adds an optional one second delay after each method call


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

        FORWARD_RIGHT, //Diagonal right
        FORWARD_LEFT, //Diagonal left
        BACKWARD_RIGHT,
        BACKWARD_LEFT
    }

    public AutonomousDriveClassV3(AdvancedRobot robot) throws InterruptedException{
        this.robot = robot;
        this.opMode = robot.getLinearVisionOpMode();
        ftcVisionManager = new FTCVisionManager(opMode,analysisMethod);
        try {
            servoControllerLib = new ServoControllerLib(opMode.hardwareMap.servo.get(servoName), servoInitPosition);
        } catch (IllegalArgumentException e){
            opMode.telemetry.addData("ERROR","Unable to find Button Pusher! Error: " + e);
        }
        try {
            lightSensor = opMode.hardwareMap.opticalDistanceSensor.get(lightSensorName);
        } catch (IllegalArgumentException e){
            opMode.telemetry.addData("ERROR","Unable to find Light Sensor! Error: " + e);
        }
        opMode.telemetry.update();
        timer = new ElapsedTime();
    }

    private void delayCall(){
        timer.reset();
        while(opMode.opModeIsActive() && timer.seconds() < 1){
            opMode.telemetry.addData("Autonomous","Delaying one second");
            opMode.telemetry.update();
        }
    }

    public void setDelayAfterEachCall(boolean delay){
        delayAfterEachCall = delay;
    }

    private double[] decodeDirection(Direction direction,double speed){
        double right = 0,left = 0,strafe = 0;
        switch (direction){
            case FORWARD:
                right = -speed;
                left = speed;
                break;
            case BACKWARD:
                right = speed;
                left = -speed;
                break;
            case RIGHT:
                strafe = speed;
                break;
            case LEFT:
                strafe = -speed;
                break;
            case FORWARD_RIGHT:
                right = -speed;
                left = speed;
                strafe = speed;
                break;
            case FORWARD_LEFT:
                right = -speed;
                left = speed;
                strafe = -speed;
                break;
            case BACKWARD_RIGHT:
                right = speed;
                left = -speed;
                strafe = speed;
                break;
            case BACKWARD_LEFT:
                right = speed;
                left = -speed;
                strafe = -speed;
                break;
            default:
                right = speed;
                left = -speed;
                break;
        }
        double[] array = {right,left,strafe};
        return array;
    }

    public void driveSetTime(double time, Direction direction, double speed) throws InterruptedException{
        timer.reset();
        double[] speeds = decodeDirection(direction,speed);
        while (opMode.opModeIsActive() && timer.seconds() < time) {
            robot.getRightSet().setPower(speeds[0]);
            robot.getLeftSet().setPower(speeds[1]);
            robot.getStrifeMotor().setPower(speeds[2]);
            opMode.telemetry.addData("Autonomous ","Moving " + direction);
            opMode.telemetry.update();
        }
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
        robot.getStrifeMotor().setPower(0);
        if (delayAfterEachCall)
            delayCall();
        opMode.telemetry.addData("Autonomous","Done");
        opMode.telemetry.update();
    }

    private void encoderDrive() throws Exception{
        //Need encoders
        //Use this for methods that need distance
        throw new Exception("encoderDrive is not yet implemented");
        //// FIXME: 4/17/2017
    }


    public void driveSetDistance() throws Exception{
        //Need encoderDrive
        throw new Exception("driveSetDistance is not yet implemented");
        //// FIXME: 4/17/2017
    }

    public void pointTurn() throws Exception{
        //Need encoderDrive
        throw new Exception("turn is not yet implemented");
        //// FIXME: 4/17/2017
    }

    public void swingTurn() throws Exception{
        //Need encoderDrive
        //Set the right or left wheels power to 0, it shouldnt be jumpy bc omni wheels dood
        throw new Exception("swingTurn is not yet implemented");
        //// FIXME: 4/17/2017
    }

    public void driveTillLine(Direction direction, double speed, double timeOut) throws InterruptedException{
        timer.reset();
        double reflectance = lightSensor.getLightDetected();
        double[] speeds = decodeDirection(direction,speed);
        while(timer.seconds() < timeOut && reflectance < WhiteLineLightLevel && opMode.opModeIsActive()){ //> or <?
            reflectance = lightSensor.getLightDetected();
            robot.getRightSet().setPower(speeds[0]);
            robot.getLeftSet().setPower(speeds[1]);
            robot.getStrifeMotor().setPower(speeds[2]);
            opMode.telemetry.addData("Autonomous","Searching for white line....");
            opMode.telemetry.update();
        }
        if (delayAfterEachCall)
            delayCall();
        opMode.telemetry.addData("Autonomous","Done");
        opMode.telemetry.update();
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
        robot.getStrifeMotor().setPower(0);
    }

    public void readAndSetServo(){
        //TODO
    }


    public void arcMoveForTime(Direction direction1,Direction direction2, double time){
        //Im gonna let Kenneth do this, I dont do circles very well

    }
    public void ultimateMove(double xDist, double yDist, double turn) throws Exception {
        throw new Exception("Not implemented yet: Get rekt m8");
        //// FIXME: 4/17/2017
    }



}
