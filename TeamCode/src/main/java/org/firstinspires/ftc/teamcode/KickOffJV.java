package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 9/9/2017.
 */

@TeleOp(name = "JV Kickoff")
public class KickOffJV extends OpMode {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DcMotor frontMotor;
    private Gamepad gp1;

    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        frontMotor = hardwareMap.dcMotor.get("frontMotor");
        gp1 = gamepad1;
    }

    @Override
    public void loop() {
        float xPower = gp1.right_stick_x;
        float yPower = gp1.right_stick_y;

        float leftPwr = xPower + yPower;

        rightMotor.setPower(xPower - yPower);
        leftMotor.setPower(leftPwr);

        if (gp1.right_bumper){
            frontMotor.setPower(.1);
        }
        if (gp1.left_bumper){
            frontMotor.setPower(-.1);
        }
        if (!(gp1.right_bumper || gp1.left_bumper)) {
            frontMotor.setPower(0);
        }
    }
}
