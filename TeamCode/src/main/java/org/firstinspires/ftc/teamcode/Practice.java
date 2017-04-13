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
        g1 = gamepad1;
        motor1 = hardwareMap.dcMotor.get("test");
        motor2 = hardwareMap.dcMotor.get("test");
        motor3 = hardwareMap.dcMotor.get("test");
        motor4 = hardwareMap.dcMotor.get("test");
        motor5 = hardwareMap.dcMotor.get("test");
    }

    @Override
    public void loop() {
        motor1.setPower(-g1.right_stick_y);
        motor2.setPower(g1.left_stick_y);
        motor3.setPower(-g1.right_stick_y);
        motor4.setPower(g1.left_stick_y);
        motor5.setPower(g1.right_stick_x);

    }
}
