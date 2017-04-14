package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 4/12/2017.
 *
 * TODO: Implement wheeled shooter motors, I wasnt sure on the set up so I skipped that
 *
 */

public class AdvancedRobot extends BasicRobot {
    private DcMotor strifeMotor;

    public AdvancedRobot(DcMotor rightSet,DcMotor leftSet,DcMotor strifeMotor, Gamepad g1){
        super(rightSet, leftSet, g1);
        this.strifeMotor = strifeMotor;
    }

    public DcMotor getStrifeMotor(){ return strifeMotor; }

    public void drive(){ getDriveTrain().driveWithGamepad(); }
    public void reverseDrive(){ getDriveTrain().reverseGamepad(); }
}
