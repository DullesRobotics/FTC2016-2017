package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.io.IOException;

/**
 * Created by awsomeaustin12 on 4/28/2017.
 */
@Autonomous(name="Encoder test")
public class EncoderDrive extends OpMode{
    DcMotor motorLeft;
    private int ENCODER_TICKS_PER_REVOLUTION = 1440;

    @Override
    public void init() {
        this.motorLeft = this.hardwareMap.dcMotor.get("motor");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop(){
        int denc = Math.round(2 * 1440);
        this.motorLeft.setTargetPosition(this.motorLeft.getCurrentPosition() + denc);
        this.motorLeft.setPower(.5);
        while (this.motorLeft.isBusy()){
            telemetry.addData("Current Position: ",this.motorLeft.getCurrentPosition());
            telemetry.update();
        }
        this.motorLeft.setPower(0);
    }
}
