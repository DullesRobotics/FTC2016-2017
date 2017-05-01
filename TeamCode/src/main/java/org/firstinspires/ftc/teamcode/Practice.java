package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 4/11/2017.
 */

@Disabled
@TeleOp(name = "Practice Op Mode")
public class Practice extends OpMode
{
    Gamepad g1;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor motor5;
    @Override
    public void init() {


    }

    @Override
    public void loop() {
        motor1.setPower(-g1.right_stick_y);

    }
}
