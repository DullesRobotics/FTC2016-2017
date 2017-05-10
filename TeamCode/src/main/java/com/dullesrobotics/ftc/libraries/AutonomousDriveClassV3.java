package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LightSensor;
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
 *
 *
 */

public class AutonomousDriveClassV3 {
    private String lightSensorName = "EOPD"; //light sensor's name
    private String servoName = "buttonServo"; //button pusher servo's name

    private double WhiteLineLightLevel = 0.125; //White line light level

    private double TicksPerRevolution = 1440;
    private double wheelRadius = 20;
    private double TicksPerCentimeter = 1/((Math.PI * 2 * wheelRadius)/TicksPerRevolution);

    private double robotWheelDistance = 0;

    private Beacon.AnalysisMethod analysisMethod = Beacon.AnalysisMethod.FAST; //Beacon analysis method...self-explanatory
    private int servoInitPosition = ServoControllerLib.SERVORIGHT; //Initial servo position

    private boolean delayAfterEachCall = true; //Adds an optional one second delay after each method call


    /**
     *        When you're modifying code below, keep it simple and make
     *        it take in as few parameters as possible.
     *  TODO: Unless you're at a contest, in which case, DO WHATEVER.
     *
     *  --Karim Karim
     *
     * */
    private LinearVisionOpMode opMode = null;
    private AdvancedRobot robot = null;
    private FTCVisionManager ftcVisionManager = null;
    private ServoControllerLib servoControllerLib = null;
    private LightSensor lightSensor = null;
    private ElapsedTime timer = null;

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,

        FORWARD_RIGHT, //Diagonal right
        FORWARD_LEFT, //Diagonal left
        BACKWARD_RIGHT,
        BACKWARD_LEFT,
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

    public void delayAutonomous(double time){
        ElapsedTime timer2 = new ElapsedTime();
        timer2.reset();
        while(opMode.opModeIsActive() && timer2.seconds() < time){
            opMode.telemetry.addData("Autonomous","Delaying " + time + " seconds.");
            opMode.telemetry.update();
        }
    }

    public void setDelayAfterEachCall(boolean delay){
        delayAfterEachCall = delay;
    }

    private double[] decodeDirection(Direction direction){
        double right = 0,left = 0,strafe = 0;
        switch (direction){
            case FORWARD:
                right = -1;
                left = 1;
                break;
            case BACKWARD:
                right = 1;
                left = -1;
                break;
            case RIGHT:
                strafe = 1;
                break;
            case LEFT:
                strafe = -1;
                break;
            case FORWARD_RIGHT:
                right = -1;
                left = 1;
                strafe = 1;
                break;
            case FORWARD_LEFT:
                right = -1;
                left = 1;
                strafe = -1;
                break;
            case BACKWARD_RIGHT:
                right = 1;
                left = -1;
                strafe = 1;
                break;
            case BACKWARD_LEFT:
                right = 1;
                left = -1;
                strafe = -1;
                break;
            default:
                right = 0;
                left = 0;
                strafe = 0;
                break;
        }
        double[] array = {right,left,strafe};
        return array;
    }

    public void driveSetTime(double time, Direction direction, double speed) throws InterruptedException{
        timer.reset();
        double[] decodedDirection = decodeDirection(direction);
        while (opMode.opModeIsActive() && timer.seconds() < time) {
            robot.getRightSet().setPower(decodedDirection[0] * speed);
            robot.getLeftSet().setPower(decodedDirection[1] * speed);
            robot.getStrafeMotor().setPower(decodedDirection[2] * speed);
            opMode.telemetry.addData("Autonomous ","Moving " + direction);
            opMode.telemetry.update();
        }
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
        robot.getStrafeMotor().setPower(0);
        if (delayAfterEachCall)
            delayCall();
        opMode.telemetry.addData("Autonomous","Done");
        opMode.telemetry.update();
    }

    public void encoderDrive(double speed, double leftCM, double rightCM, double time) throws InterruptedException{
        robot.getRightSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getLeftSet().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getRightSet().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftSet().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
        robot.getRightSet().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftSet().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int rightTarget = robot.getRightSet().getCurrentPosition() + (int)(rightCM * TicksPerCentimeter);
        int leftTarget = robot.getLeftSet().getCurrentPosition() + (int)(leftCM * TicksPerCentimeter);
        robot.getRightSet().setTargetPosition(rightTarget);
        robot.getLeftSet().setTargetPosition(leftTarget);
        timer.reset();
        robot.getRightSet().setPower(Math.abs(speed));
        robot.getLeftSet().setPower(Math.abs(speed));
        while (opMode.opModeIsActive() && timer.seconds() < time && robot.getRightSet().isBusy() && robot.getLeftSet().isBusy()){
            opMode.telemetry.addData("AUTONOMOUS","Target Position(R,L): " + rightTarget + "," + leftTarget);
            opMode.telemetry.addData("AUTONOMOUS","Current Position(R,L): " + robot.getRightSet().getCurrentPosition() + "," + robot.getLeftSet().getCurrentPosition());
            opMode.telemetry.addData("Time: ", timer.seconds());
            opMode.telemetry.update();
        }
        robot.getRightSet().setPower(0);
        robot.getLeftSet().setPower(0);
    }
    /*public boolean encoderDrive(double speed, double leftCM, double rightCM, double time){
                int newLeftTarget;
                int newRightTarget;
                timer.reset();
                boolean hitTimeOut = false;
                if (opMode.opModeIsActive()) {
                newLeftTarget = robot.getLeftSet().getCurrentPosition() + (int)(leftCM * TicksPerCentimeter);
                newRightTarget = robot.getRightSet().getCurrentPosition() + (int)(rightCM * TicksPerCentimeter);
                robot.getLeftSet().setTargetPosition(newLeftTarget);
                robot.getRightSet().setTargetPosition(newRightTarget);
                robot.getLeftSet().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.getRightSet().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                timer.reset();
                robot.getLeftSet().setPower(Math.abs(speed));
                robot.getRightSet().setPower(Math.abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                opMode.telemetry.addData("opModeisActive",opMode.opModeIsActive());
                opMode.telemetry.addData("runtime.seconds() < timeoutS",(timer.seconds() < time));
                opMode.telemetry.addData("at least one motor Busy",(robot.getLeftSet().isBusy() || robot.getRightSet().isBusy()));
                opMode.telemetry.update();
                while (opMode.opModeIsActive() && (robot.getLeftSet().isBusy() || robot.getRightSet().isBusy() || timer.seconds() < time)) {
                    delay(1);
                    // Display it for the driver.
                    opMode.telemetry.addData("Targets",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    opMode.telemetry.addData("CurrentPos",  "Running at %7d :%7d",
                            robot.getLeftSet().getCurrentPosition(),
                            robot.getRightSet().getCurrentPosition());
                    opMode.telemetry.update();
                    if(timer.seconds() > time){
                        hitTimeOut = true;
                        return hitTimeOut;
                    }
                    if (!robot.getLeftSet().isBusy() && !robot.getRightSet().isBusy()){
                        return false;
                    }
                }
                // Stop all motion;
                robot.getLeftSet().setPower(0);
                robot.getRightSet().setPower(0);
                // Turn off RUN_TO_POSITION
                robot.getLeftSet().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.getRightSet().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                    return hitTimeOut;
            }
            return hitTimeOut;
    }*/

    public void driveSetDistance(double speed, double distance, double time) throws InterruptedException{
        double toCm = distance * 2.54;
        encoderDrive(speed,distance,distance,time);
    }

    public void driveSetDistanceCM(double speed, double distance, double time) throws InterruptedException{
        encoderDrive(speed,distance,distance,time);
    }

    public void driveWithinTime(double distance, double time) throws InterruptedException{
        double speed = distance/time;
        encoderDrive(speed,distance,distance,time);
    }

    public void pointTurn(double speed, double degrees, double time) throws Exception{
        encoderDrive(speed,(degrees/360)*robotWheelDistance * Math.PI,-(degrees/360)*robotWheelDistance * Math.PI,time);
    }

    public void swingTurn(double speed, double degrees, double time) throws Exception{
        encoderDrive(speed,(degrees/360)* 2 * robotWheelDistance * Math.PI,0,time);
    }

    public void driveTillLine(Direction direction, double speed, double timeOut) throws InterruptedException{
        if (lightSensor != null) {
            timer.reset();
            double reflectance = lightSensor.getLightDetected();
            double[] decodedDirection = decodeDirection(direction);
            while (timer.seconds() < timeOut && reflectance < WhiteLineLightLevel && opMode.opModeIsActive()) { //> or <?
                reflectance = lightSensor.getLightDetected();
                robot.getRightSet().setPower(decodedDirection[0] * speed);
                robot.getLeftSet().setPower(decodedDirection[1] * speed);
                robot.getStrafeMotor().setPower(decodedDirection[2] * speed);
                opMode.telemetry.addData("AUTONOMOUS", "Searching for white line....");
                opMode.telemetry.update();
            }
            if (delayAfterEachCall)
                delayCall();
            opMode.telemetry.addData("AUTONOMOUS", "Done");
            opMode.telemetry.update();
            robot.getRightSet().setPower(0);
            robot.getLeftSet().setPower(0);
            robot.getStrafeMotor().setPower(0);
        } else {
            opMode.telemetry.addData("ERROR","LightSensor not found; Failed method 'driveTillLine'");
            opMode.telemetry.update();
        }
    }

    public String getVisionAnalysis() throws InterruptedException{
        return ftcVisionManager.readBeacon(7,3);
    }

    public void setServo(String teamOn) throws InterruptedException{
        if (servoControllerLib != null) {
            if (teamOn.toLowerCase().equals("blue")) {
                if (getVisionAnalysis().toLowerCase().equals("redblue")) {
                    servoControllerLib.setDegrees(servoControllerLib.SERVORIGHT);
                } else {
                    servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
                }
            } else {
                if (getVisionAnalysis().toLowerCase().equals("bluered")) {
                    servoControllerLib.setDegrees(servoControllerLib.SERVORIGHT);
                } else {
                    servoControllerLib.setDegrees(ServoControllerLib.SERVOLEFT);
                }
            }
            opMode.telemetry.addData("Autonomous", "Successfully set servo position");
            opMode.telemetry.update();
        } else {
            opMode.telemetry.addData("ERROR","ServoControllerLib not instantiated; Failed method 'setServo'");
            opMode.telemetry.update();
        }
    }

    public void shoot(double time){
        try {
            timer.reset();
            while (opMode.opModeIsActive() && timer.seconds() < time) {
                robot.getShooterMotor().setPower(1);
            }
            robot.getShooterMotor().setPower(0);
            opMode.telemetry.addData("Autonomous","Successfully shot!");
            opMode.telemetry.update();
        } catch (NullPointerException e){
            opMode.telemetry.addData("ERROR","Failed to shoot!");
            opMode.telemetry.update();
        }
    }

    public void shoot(){
        shoot(3);
    }


    public void arcMoveForTime(Direction direction1,Direction direction2, double time){
        //Im gonna let Kenneth do this, I don't do circles very well

    }
    public void ultimateMove(double xDist, double yDist, double turn) throws InterruptedException {
        throw new InterruptedException("Not implemented yet: Get rekt m8");
        //// FIXME: 4/17/2017
    }
}