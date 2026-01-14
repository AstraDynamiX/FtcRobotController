package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Random;

/**
 * The purpose of this class is to track how the motor reacts to new inputs
 * We give the motor a random speed for a random amount of time
 * and write encoder values every loop call.

 * It then outputs a table with this row structure:
 * TIMESTAMP  MOTOR INPUT (the number we put in SetVelocity)  ENCODER OUTPUT
 * This table is fed into Matlab, which uses it to generate a model
 * of our motor, which is then used to generate PID coefficients mathematically
 */

@Configurable
@TeleOp
public class MotorModelFormer extends OpMode
{
    private FileWriter writer;
    Random rand = new Random();
    ElapsedTime now = new ElapsedTime();

    MotorEx motor;

    private long lastLogTime;
    private long input = 0;
    private int inputTime = 100;
    private long inputTimeCheck = 0;


    @Override
    public void init()
    {
        //File setup
        //Control Hub saves accessible files in this folder
        File file = new File("/sdcard/FIRST/motorData.csv");

        try {
            writer = new FileWriter(file, false); // overwrite each run
            writer.flush();
        }
        catch (IOException e)
        {RobotLog.e("Failed to open log file: " + e.getMessage());}

        //Motor setup
        motor = new MotorEx(hardwareMap, "flywheel", 28, 6000);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(0, 0, 0); //No PID because that's what we're trying to generate
        motor.setInverted(false);

        now.reset();
    }

    @Override
    public void loop()
    {
        if (now.milliseconds() - inputTimeCheck >= inputTime)
        {
            input = rand.nextInt(28 * 75); //Degrees in a revolution * motor practical max revolutions per second
            inputTime = 50 + rand.nextInt(1200); //milliseconds (min. 50, max. 1250)
            inputTimeCheck = (long) now.milliseconds();

            motor.setVelocity(input);
        }

        if (writer != null && now.milliseconds() - lastLogTime >= 50) //20 Hz
        {
            try {
                writer.write(
                    now + "," +
                    input + "," +
                    motor.getVelocity() + "\n"
                );
            }
            catch (IOException e)
            {RobotLog.e("File write failed: " + e.getMessage());}
            lastLogTime = (long) now.milliseconds();
        }

        telemetry.addData("ENCODER", motor.motor.getCurrentPosition());
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