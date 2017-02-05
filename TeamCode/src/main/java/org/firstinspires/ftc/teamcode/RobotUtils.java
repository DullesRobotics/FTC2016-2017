package org.firstinspires.ftc.teamcode;

/**
 * Created by kk200 on 2/4/2017.
 */

import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class RobotUtils {
    public static void servo(Servo s, int angle, boolean condition){
        s.setPosition(angle);
    }

    public void rotateServo() {}
    public void turn(int degrees) {}
    public void move(int inches, int power) {}
}
