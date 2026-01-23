package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Locale;
import java.util.Random;

/**
 * The purpose of this class is to track how the chassis reacts to new inputs
 * We give the chassis a random speed for a random amount of time
 * and write encoder values every loop call.
 * Repeat this for every axis (axial, lateral, yaw)

 * It then outputs a table with this row structure:
 * TIMESTAMP  AXIAL CMD  LATERAL CMD  YAW CMD  AXIAL ODO POD (X)  LATERAL ODO POD (Y)  IMU
 * This table is fed into Matlab, which uses it to generate a model
 * of our motor, which is then used to generate PID coefficients mathematically
 */

@TeleOp(group = "modelFormers")
public class ChassisModelFormer extends OpMode
{
    private final int TICKS_PER_REV = 28;
    private final double BOUNDS = 37; //in - how much room the robot has

    MotorEx leftFrontWheel;
    MotorEx rightFrontWheel;
    MotorEx leftBackWheel;
    MotorEx rightBackWheel;
    GoBildaPinpointDriver pinpoint;

    private FileWriter writer;
    Random rand = new Random();
    ElapsedTime now = new ElapsedTime();

    private long lastLogTime;
    private int inputTime = 100;
    private long inputTimeCheck = 0;

    private int currentAx = 0;
    private int[] axValues = new int[]{0, 0, 0};
    private boolean pause = false;
    private int preventAggressiveBraking = 1;

    private boolean aHeld = false;
    private boolean bHeld = false;


    @Override
    public void init()
    {
        // ------ File setup ------
        //Control Hub saves accessible files in this folder
        File file = new File("/sdcard/FIRST/chassisData.csv");

        try {
            writer = new FileWriter(file, false);
            writer.flush();
        }
        catch (IOException e)
        {RobotLog.e("Failed to open log file: " + e.getMessage());}

        now.reset();

        // ------ Motor setup ------
        leftFrontWheel = initMotor(hardwareMap, true, "flWheel");
        rightFrontWheel = initMotor(hardwareMap, false, "frWheel");
        leftBackWheel = initMotor(hardwareMap, true, "blWheel");
        rightBackWheel = initMotor(hardwareMap, false, "brWheel");

        // ------ Odometry computer setup ------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(4.92, 1.81, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void init_loop()
    {
        pinpoint.update();
        Pose2D position = pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH),
                position.getHeading(AngleUnit.DEGREES));
        telemetry.addData("POSITION", data);
    }

    private MotorEx initMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, TICKS_PER_REV, 435);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setInverted(inverted);
        motor.setCachingTolerance(0.075);
        return motor;
    }

    @Override
    public void loop()
    {
        pinpoint.update();
        double position = 0;
        switch (currentAx)
        {
            case 0: position = pinpoint.getPosX(DistanceUnit.INCH); break;
            case 1: position = pinpoint.getPosY(DistanceUnit.INCH); break;
        }
        telemetry.addData("POSITION", position);

        //Switch axes
        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            currentAx++;
            if (currentAx >= 3) {currentAx = 0;}
        }
        if (!gamepad1.a) {aHeld = false;}

        telemetry.addData("CURRENT AX", currentAx);

        //Space is limited and pausing and repositioning the robot may be needed
        if (gamepad1.b && !bHeld)
        {
            bHeld = true;
            pause = !pause;
        }
        if (!gamepad1.b) {bHeld = false;}

        if (pause)
        {
            telemetry.addData("", "PAUSED");
            PowerWheels(0, 0, 0);
            return;
        }

        int inBounds;
        if (position < -BOUNDS) {inBounds = 0;} //too far back
        else if (position > BOUNDS) {inBounds = 2;} //too far front
        else {inBounds = 1;} //in bounds

        if (now.milliseconds() - inputTimeCheck >= inputTime || inBounds != 1)
        {
            //Degrees in a revolution * motor practical max revolutions per second
            int input = 0;
            //This, well, prevents aggressive braking that can mess up encoders and isn't really practical
            //(I don't plan on doing wheelies during autonomous)
            if (preventAggressiveBraking == 0)
            {
                input = -(TICKS_PER_REV) + rand.nextInt(TICKS_PER_REV);
                preventAggressiveBraking = 1;
            }
            else if (preventAggressiveBraking == 2)
            {
                input = rand.nextInt(TICKS_PER_REV);
                preventAggressiveBraking = 1;
            }
            else
            {input = -(TICKS_PER_REV * 6 * inBounds) + rand.nextInt(2 * TICKS_PER_REV * 6);}
            inputTime = 50 + rand.nextInt(750); //milliseconds (min. 50, max. 800)
            inputTimeCheck = (long) now.milliseconds();

            if (input < -TICKS_PER_REV * 4) {preventAggressiveBraking = 0;} //going backwards very fast
            else if (input > TICKS_PER_REV * 4) {preventAggressiveBraking = 2;} //going forwards very fast

            axValues = new int[]{0, 0, 0};
            axValues[currentAx] = Math.toIntExact(input);

            PowerWheels(axValues[0], axValues[1], axValues[2]);
        }

        if (writer != null && now.milliseconds() - lastLogTime >= 50) //20 Hz
        {
            try {
                writer.write(
                        now.seconds() + "," +
                        axValues[0] + "," +
                        axValues[1] + "," +
                        axValues[2] + "," +
                        pinpoint.getVelX(DistanceUnit.INCH) + "," +
                        pinpoint.getVelY(DistanceUnit.INCH) + "," +
                        pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) + '\n'
                );
            }
            catch (IOException e)
            {RobotLog.e("File write failed: " + e.getMessage());}
            lastLogTime = (long) now.milliseconds();
        }

        if (preventAggressiveBraking != 1) {telemetry.addData("", "PREVENT AGGRESSIVE BRAKING");}
    }

    public void PowerWheels(double axial, double lateral, double yaw)
    {
        leftFrontWheel.setVelocity(axial - lateral - yaw);
        leftBackWheel.setVelocity(axial + lateral - yaw);
        rightFrontWheel.setVelocity(axial + lateral + yaw);
        rightBackWheel.setVelocity(axial - lateral + yaw);
    }

    @Override
    public void stop()
    {
        if (writer != null)
        {
            try {
                writer.flush();
                writer.close();
            }
            catch (IOException e)
            {RobotLog.e("Failed to close log file: " + e.getMessage());}
        }
    }
}