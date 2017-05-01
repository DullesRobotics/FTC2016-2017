package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Created by kk200 on 4/28/2017.
 */

public class EncoderDrive extends LinearVisionOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor;
        motor=hardwareMap.dcMotor.get("motor");
        waitForStart();

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(10000);
        motor.setPower(.5);
    }
}
