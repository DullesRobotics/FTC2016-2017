package com.dullesrobotics.ftc.libraries;

import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Kenneth on 1/8/2017.
 */

public class AutonomousDrive {
    BasicRobot robot;
    final double ENCODERTICKSPERREVOLUTION = 1478.4;
    final double CIRCUMFERENCEOFWHEELCENTIMETERS = Math.PI*9.6;
    final double TICKSPERCENTIMETER = CIRCUMFERENCEOFWHEELCENTIMETERS/ENCODERTICKSPERREVOLUTION;
    //SensorListener sensorListener;
    //float heading;
/*
    public AutonomousDrive(BasicRobot r, SensorListener s) {
        robot = r;
        sensorListener = s;
    }
*/
    public AutonomousDrive(BasicRobot r) {
        robot = r;
    }

    public void driveStraightADistanceWithEncoders(double centimeters){
        int ticks = (int) (centimeters*TICKSPERCENTIMETER);
        resetEncoders();
        setRUNWITHENCODERS();
        int firstFifth = (int) (ticks*.2);
        int lastFifth = (int)(ticks*.8);
        final double MAXPOWER = 0.75;
        double power = 0;
        while(robot.getBLM().getCurrentPosition()<=ticks || robot.getBRM().getCurrentPosition()<=ticks){
            if(robot.getBRM().getCurrentPosition()<firstFifth && robot.getBRM().getCurrentPosition()<firstFifth){
                power = getMaxCurrentPos()*5.0/ticks*MAXPOWER;
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            } else if(robot.getBLM().getCurrentPosition()>=lastFifth||robot.getBRM().getCurrentPosition()>=lastFifth){
                power = MAXPOWER - (getMaxCurrentPos()/ticks*MAXPOWER);
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            }else{
                power = MAXPOWER;
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            }
        }
    }

    public void pointTurn(double angleInDeg){
        double percentOfCircle = angleInDeg/360.0;
        double turnRadiusCM = 5*2.54;
        double ticksNeededToTurn = 2*Math.PI*turnRadiusCM*TICKSPERCENTIMETER*percentOfCircle;
        resetEncoders();
        setRUNTOPOSITION();
        robot.getBRM().setTargetPosition((int) (ticksNeededToTurn/2.0));
        robot.getBLM().setTargetPosition((int) (ticksNeededToTurn/2.0));
        robot.getBLM().setPower(0.2);
        robot.getBRM().setPower(0.2);
    }

    public void swingTurn(double angleInDeg){
        double percentOfCircle = (angleInDeg>0)? angleInDeg:angleInDeg*-1.0;
        double turnRadiusCM = 5*2.54;
        double ticksNeededToTurn = 2*Math.PI*turnRadiusCM*TICKSPERCENTIMETER*percentOfCircle;
        resetEncoders();
        setRUNTOPOSITION();
        if(angleInDeg>0){
            //Turn Right
            robot.getBLM().setTargetPosition((int)(ticksNeededToTurn));
        }else{
            robot.getBRM().setTargetPosition((int)(ticksNeededToTurn));
        }
        robot.getBLM().setPower(0.2);
        robot.getBRM().setPower(0.2);
    }

    public void driveStraightForSetTime(double seconds){
        long millis = (long) (seconds*1000.0);
        long start = System.currentTimeMillis();
        resetEncoders();
        setRUNWITHENCODERS();
        long firstFifth = (long) ((millis*.2)+start);
        long lastFifth = (long)(millis*.8)+start;
        final double MAXPOWER = 0.75;
        double power = 0;
        long end = start + millis;
        while(System.currentTimeMillis()<= end){
            long curTime = System.currentTimeMillis();
            if(curTime<=firstFifth){
                power = (((double)(curTime)-((double)start))*5.0/((double)(millis)))*MAXPOWER;
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            } else if(curTime >= lastFifth){
                power = MAXPOWER - (((double)(curTime)-((double)start))*5.0/((double)(millis)))*MAXPOWER;
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            }else{
                power = MAXPOWER;
                robot.getBRM().setPower(power);
                robot.getBLM().setPower(power);
            }
        }
    }

    public int getMaxCurrentPos(){
        return Math.max(robot.getBLM().getCurrentPosition(),robot.getBRM().getCurrentPosition());
    }





    public void setRUNWITHENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setRUNWITHOUTENCODERS(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRUNTOPOSITION(){
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetEncoders(){
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
