package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 2/7/2017.
 */


public class RobotWithFlickerShooterandMecanum extends BasicRobot {

    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor BLM;
    private DcMotor BRM;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private DcMotor shooterMotor;
    private DcMotor lift;

    public RobotWithFlickerShooterandMecanum(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gamepad g1, Gamepad g2, DcMotor shooter){

        FLM = frontLeft;
        FRM = frontRight;
        BLM = backLeft;
        BRM = backRight;
        gamepad1 = g1;
        gamepad2 = g2;
    }


    public DcMotor getShooterMotor() {
        return shooterMotor;
    }

    public void setShooterMotor(DcMotor shooterMotor) {
        this.shooterMotor = shooterMotor;
    }

    public void turnForwards(float power){shooterMotor.setPower(power);}
    public void turnBackwards(float power){
        shooterMotor.setPower(power);
    }
    public void turnForwards(){
        shooterMotor.setPower(1.0);
    }
    public void turnBackwards(){
        shooterMotor.setPower(-1.0);
    }
    public void stopShooter(){
        shooterMotor.setPower(0.0);
    }
    public void moveRight(){ }
    public void moveLeft(){}
    public void moveUP(){ }
    public void moveDOWN(){}



}
