package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.dullesrobotics.ftc.libraries.ArcadeDrive;
import com.dullesrobotics.ftc.libraries.RobotWithFlickerShooter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Timer;

/**
 * Created by Kenneth on 1/4/2017.
 */
@Autonomous(name="BLUE Better Autonomous")
public class BetterAutonomousBlue extends LinearOpMode{
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






        //Shoot 2nd Ball
        //firgure out later






        //Move froward 6 inches
        int DRIVE_SPEED = 1;
        encoderDrive(DRIVE_SPEED,   6, -6, 2.0);








        //Turn ~45deg RIGHT (BLUE TURNS RIGHT TO BEACOM) (RED TURNS COUNTERCLOWISE OR LEFT)
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopAndResetEncoders();
        robot.getBLM().setTargetPosition(distToEncoderTicks(2*3.1415*12*2.54/8));  //Assuming Robot's turning radius is 12in b/c dist between wheels is ~12in & 45deg is 360/8
        while(robot.getBLM().isBusy()){}










        // Keep Going forwards till EOPD detects line
        OpticalDistanceSensor distanceSensor;
        distanceSensor = hardwareMap.opticalDistanceSensor.get("EOPD");
        double tileLight = distanceSensor.getLightDetected();
       //the initialization of whitelight is what is recommended but TEST IT AGAINST WHITE TAPE or WHITE PAPER and reset the value adjusted to what you get
        double whiteLight  = tileLight +50;

        while(distanceSensor.getLightDetected() <= (whiteLight)){
            robot.getBLM().setPower(1);
            robot.getBRM().setPower(1);
        }
        robot.getBLM().setPower(0);
        robot.getBRM().setPower(0);











        //turn 45
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopAndResetEncoders();
        robot.getBLM().setTargetPosition(distToEncoderTicks(2*3.1415*12*2.54/8));

        //Assuming Robot's turning radius is 12in b/c dist between wheels is ~12in & 45deg is 360/8
        while(robot.getBLM().isBusy()){}








        //move half the distance of the robot
        double robotFullLength = 12;  //actually measure exact length of our robots base
        encoderDrive(DRIVE_SPEED,(robotFullLength/2), (-1*(robotFullLength/2)), 2.0);




        //turn 45
        robot.getBLM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBRM().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stopAndResetEncoders();
        robot.getBLM().setTargetPosition(distToEncoderTicks(2*3.1415*12*2.54/8));
        //Assuming Robot's turning radius is 12in b/c dist between wheels is ~12in & 45deg is 360/8
        while(robot.getBLM().isBusy()){}


       //now take in information from the camera
        








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
}
