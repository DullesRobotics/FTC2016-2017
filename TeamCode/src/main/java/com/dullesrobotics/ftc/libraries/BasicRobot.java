package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;


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
    private DcMotor rightSet;
    private DcMotor leftSet;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private TeleOpDrivetrain driveTrain;
    private OpMode opMode;
    private LinearVisionOpMode linearVisionOpMode;

    final String rightSetName = "rightMotors"; /** CHANGE THIS */
    final String leftSetName = "leftMotors"; /** CHANGE THIS */

    public BasicRobot(OpMode opMode){
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.rightSet = opMode.hardwareMap.dcMotor.get(rightSetName);
        this.leftSet = opMode.hardwareMap.dcMotor.get(leftSetName);
    }

    public BasicRobot(LinearVisionOpMode opMode){
        this.linearVisionOpMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.rightSet = opMode.hardwareMap.dcMotor.get(rightSetName);
        this.leftSet = opMode.hardwareMap.dcMotor.get(leftSetName);
    }

    public void setDriveTrain(TeleOpDrivetrain driveTrain) {
        this.driveTrain = driveTrain;
    }
    public TeleOpDrivetrain getDriveTrain() {
        return driveTrain;
    }

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
    public DcMotor getRightSet(){ return rightSet;}
    public DcMotor getLeftSet(){ return leftSet;}

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
    public void setRightSet(DcMotor rightSet){ this.rightSet = rightSet; }
    public void setLeftSet(DcMotor leftSet){ this.leftSet = leftSet; }

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

    public OpMode getOpMode(){ return opMode; }

    public LinearVisionOpMode getLinearVisionOpMode() { return linearVisionOpMode; }
}
