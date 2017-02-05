package com.dullesrobotics.ftc.libraries;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kk200 on 1/6/2017.
 */

public class ServoControllerLib {
    private Servo buttonServo;
    public static final int SERVOLEFT = -360;
    public static final int SERVORIGHT = 360;
    public ServoControllerLib(Servo btnServ){
        buttonServo = btnServ;
    }
    public Servo[] servos;

    public ServoControllerLib(Servo btnServ, double degrees){
        buttonServo = btnServ;
        buttonServo.setPosition(degrees);
    }

    public ServoControllerLib(Servo[] servos,double degrees){
        this.servos = servos;
        for (Servo s : this.servos){
            s.setPosition(degrees);
        }
    }

    public ServoControllerLib(Servo[] servos){
        this.servos = servos;
    }

    public ServoControllerLib(Servo btnServ, Servo.Direction dir){
        buttonServo = btnServ;
        buttonServo.setDirection(dir);
    }

    public ServoControllerLib(Servo[] servos,Servo.Direction dir){
        this.servos = servos;
        for (Servo s : this.servos){
            s.setDirection(dir);
        }
    }

    public void setDegrees(double degrees){
        buttonServo.setPosition(degrees);
    }

    public void setDegrees(double degrees, int index){ servos[index].setPosition(degrees); }

    public void setDegreesAll(double degrees){
        for (Servo s:servos)
            s.setPosition(degrees);
    }

    public void setDirection(Servo.Direction dir){
        buttonServo.setDirection(dir);
    }

    public void setDirection(Servo.Direction dir, int index){
        servos[index].setDirection(dir);
    }

    public void setDirectionAll(Servo.Direction dir){
        for (Servo s:servos)
            s.setDirection(dir);
    }



    public Servo returnServo(){
        return buttonServo;
    }
}
