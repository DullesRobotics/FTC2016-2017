package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name = "Test Op Mode")
public class test1 extends OpMode{
    DcMotor right,left;
    Gamepad g;
    @Override
    public void init() {
        right = hardwareMap.dcMotor.get("rightMotors");
        left = hardwareMap.dcMotor.get("leftMotors");
        g = gamepad1;
    }

    @Override
    public void loop() {
        right.setPower(g.right_stick_y);
        left.setPower(g.left_stick_y);
    }
}
