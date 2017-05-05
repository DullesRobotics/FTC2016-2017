package com.dullesrobotics.ftc.libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/12/2017.
 *
 * TODO: Implement wheeled shooter
 *
 */

public class AdvancedRobot extends BasicRobot {
    private DcMotor strafeMotor;
    final String strafeMotorName = "strafeMotor"; /** CHANGE THIS */

    public AdvancedRobot(OpMode opMode){
        super(opMode);
        try {
            this.strafeMotor = opMode.hardwareMap.dcMotor.get(strafeMotorName);
        } catch (NullPointerException e){
            opMode.telemetry.addData("ERROR:","Failed to find Strafe Motor! " + e);
            opMode.telemetry.update();
        }
    }

    public AdvancedRobot(LinearVisionOpMode opMode){
        super(opMode);
        try {
            this.strafeMotor = opMode.hardwareMap.dcMotor.get(strafeMotorName);
        } catch (NullPointerException e){
            opMode.telemetry.addData("ERROR:","Failed to find Strafe Motor! " + e);
            opMode.telemetry.update();
        }
    }

    public DcMotor getStrafeMotor(){ //Karim likes naming stuff wrong
        return strafeMotor;
    }

    public void drive(){ getDriveTrain().driveWithGamepad(); }
    public void reverseDrive(){ getDriveTrain().reverseGamepad(); }
}
