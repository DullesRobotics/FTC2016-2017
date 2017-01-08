package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.SensorListener;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Kenneth on 1/8/2017.
 */
@Autonomous(name = "OrientationTest")
public class OrientationTest extends OpMode {
    SensorListener sensorListener = new SensorListener(this, SensorListener.PORTRAIT_VERTICAL);

    @Override
    public void init() {
        sensorListener.register();
    }

    @Override
    public void loop() {
        telemetry.addData("Delta Yaw",sensorListener.getYaw());
    }

}
