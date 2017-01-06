package org.firstinspires.ftc.teamcode;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.dullesrobotics.ftc.libraries.RobotWithWheeledShooter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Kenneth on 1/6/2017.
 */

public class EasyAutonomous extends LinearOpMode {
    final double ENCODERTICKSPERREVOLUTION = 1478.4;
    final double CIRCUMFERENCEOFWHEELCENTIMETERS = 3.1416*9.6;//ModernRobotics has generously not provided us with a CAD file for the wheel so let's use 96mm diam until we get an actual measurement
    long startTime;
    RobotWithFlickerShooter robot;
    ArcadeDrive ArcDrive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Start Timer
        startTime = System.currentTimeMillis();





        //Initialize Variables
        robot = new RobotWithFlickerShooter(hardwareMap.dcMotor.get("BLM"),hardwareMap.dcMotor.get("BRM"),gamepad1,hardwareMap.dcMotor.get("flickerShooter"));
        //Set to Arcade Drive (Autonomous Arcade Drive)
        ArcDrive = new ArcadeDrive(robot);
        robot.setDriveTrain(ArcDrive);






        //Shoot ONE Ball
        robot.turnBackwards();
        wait(5000);
        robot.stopShooter();





        //Move froward 6 inches
        int DRIVE_SPEED = 1;
        encoderDrive(DRIVE_SPEED,   6, -6, 2.0);



        //Go 6ft fwd (3 tiles)
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopAndResetEncoders();
        double distInches = 12*6;
        robot.getBLM().setTargetPosition(distToEncoderTicks(distInches*2.54));  //Assuming Robot's turning radius is 12in b/c dist between wheels is ~12in & 45deg is 360/8
        while(robot.getBLM().isBusy()||robot.getBRM().isBusy()){}



    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder // CHECK IF THIS IS CORRECT
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //CHECK IF THIS IS CORRECT
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference  // CHECK IF THIS IS CORRECT
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getBLM().getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getBRM().getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.getBLM().setTargetPosition(newLeftTarget);
            robot.getBRM().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.getBLM().setPower(Math.abs(speed));
            robot.getBRM().setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.getBLM().isBusy() && robot.getBRM().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.getBLM().getCurrentPosition(),
                        robot.getBRM().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.getBLM().setPower(0);
            robot.getBRM().setPower(0);

            // Turn off RUN_TO_POSITION
            robot.getBLM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.getBRM().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public double secsSinceStart(){
        return ((Number)(System.currentTimeMillis()-startTime)).doubleValue()/(1000.0);
    }
    public void stopAndResetEncoders() throws InterruptedException {
        robot.getBLM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBRM().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wait(1);//Example I used waited one hardware cycle but the method is deprecated
    }
    public int distToEncoderTicks(double dist){
        return (int)(dist/CIRCUMFERENCEOFWHEELCENTIMETERS*ENCODERTICKSPERREVOLUTION);
    }
}
