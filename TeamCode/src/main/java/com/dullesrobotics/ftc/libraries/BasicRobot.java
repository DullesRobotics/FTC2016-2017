package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Kenneth on 11/6/2016.
 *
 * TODO: Maybe make a new class to use instead of BasicRobot...since its not basic anymore, and has 2 wheels.
 */

public class BasicRobot {
    //Has 4 wheels
    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor BLM;
    private DcMotor BRM;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private TeleOpDrivetrain driveTrain;

    public BasicRobot(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Gamepad g1){
        this.FLM = FLM;
        this.FRM = FRM;
        this.BLM = BLM;
        this.BRM = BRM;
        this.gamepad1 = g1;
    }

    public BasicRobot(DcMotor[] motors, Gamepad g1){
        if (motors[0] != null) this.FRM = motors[0];
        if (motors[1] != null) this.FLM = motors[1];
        if (motors[2] != null) this.BRM = motors[2];
        if (motors[3] != null) this.BLM = motors[3];
        this.gamepad1 = g1;
    }


    public void setDriveTrain(TeleOpDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
    }
    public TeleOpDrivetrain getDriveTrain() {
        return driveTrain;
    }

    public void driveWithGamepad(){
        driveTrain.driveWithGamepad();
    }
    public void reverseGamepad() { driveTrain.reverseGamepad();}

    public DcMotor getBLM() {
        return BLM;
    }
    public DcMotor getBRM() {
        return BRM;
    }
    public DcMotor getFLM() {
        return FLM;
    }
    public DcMotor getFRM() {
        return FRM;
    }

    public void setFLM(DcMotor FLM) {
        this.FLM = FLM;
    }
    public void setFRM(DcMotor FRM) {
       this.FRM = FRM;
    }
    public void setBLM(DcMotor BLM) {
        this.BLM = BLM;
    }
    public void setBRM(DcMotor BRM) {
        this.BRM = BRM;
    }

    public Gamepad getGamepad1(){
        return gamepad1;
    }
    public Gamepad getGamepad2() {
        return gamepad2;
    }

    public void setGamepad1(Gamepad g1) {
        gamepad1 = g1;
    }
    public void setGamepad2(Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
    }
}
