package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Kenneth on 1/6/2017.
 */

@Autonomous(name="EOPD Test")
public class EOPDTest extends OpMode {
    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    @Override
    public void init() {
        odsSensor = hardwareMap.opticalDistanceSensor.get("EOPD");
        odsSensor.enableLed(true);

    }

    @Override
    public void loop() {
        telemetry.addData("Raw",    odsSensor.getRawLightDetected());
        telemetry.addData("Normal", odsSensor.getLightDetected());
//        telemetry.

        //telemetry.update();

    }
}
