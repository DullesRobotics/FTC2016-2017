package org.firstinspires.ftc.teamcode;


import com.dullesrobotics.ftc.mods.SensorListener;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by Kenneth on 1/8/2017.
 */
@Disabled
@Autonomous(name = "OrientationTest")
public class OrientationTest extends OpMode {
SensorListener sensorListener;

    @Override
    public void init() {
       // sensorListener = ((FtcRobotControllerActivity)hardwareMap.appContext).getSensorListener();
    }

    @Override
    public void loop() {
        //telemetry.addData("Delta Yaw",sensorListener.getYaw());
    }

}