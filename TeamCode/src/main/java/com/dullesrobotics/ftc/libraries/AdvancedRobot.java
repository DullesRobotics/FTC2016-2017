package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 4/12/2017.
 *
 * TODO: Implement wheeled shooter
 *
 */

public class AdvancedRobot extends BasicRobot {
    private DcMotor strifeMotor;
    final String strafeMotorName = "strafeMotor"; /** CHANGE THIS */

    public AdvancedRobot(OpMode opMode){
        super(opMode, opMode.gamepad1);
        this.strifeMotor = opMode.hardwareMap.dcMotor.get(strafeMotorName);
    }

    public AdvancedRobot(DcMotor rightSet,DcMotor leftSet,DcMotor strifeMotor, Gamepad g1){
        super(rightSet, leftSet, g1);
        this.strifeMotor = strifeMotor;
    }

    public DcMotor getStrifeMotor(){ return strifeMotor; }

    public void drive(){ getDriveTrain().driveWithGamepad(); }
    public void reverseDrive(){ getDriveTrain().reverseGamepad(); }
}
