package org.firstinspires.ftc.teamcode.OpModes.Readers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

/**
 * The purpose of this class is to track how the motor reacts to new inputs
 * We give the motor a random speed for a random amount of time
 * and write encoder values every loop call.

 * It then outputs a table with this row structure:
 * TIMESTAMP  MOTOR INPUT (the number we put in motor.set())  ENCODER OUTPUT
 */

@TeleOp(group = "readers")
public class MotorRandomExcitation extends OpMode
{
    private FileWriter writer;
    Random rand = new Random();
    ElapsedTime now = new ElapsedTime();

    MotorEx motor;
    MotorEx helperMotor;

    private double lastLogTime;
    private double input = 0;
    private int inputTime = 100;
    private long inputTimeCheck = 0;


    @Override
    public void init()
    {
        //File setup
        //Control Hub saves accessible files in this folder
        File file = new File("/sdcard/FIRST/motorData.csv");

        try {
            writer = new FileWriter(file, false); //Overwrite each run
            writer.flush();
        }
        catch (IOException e)
        {RobotLog.e("Failed to open log file: " + e.getMessage());}

        //Motor setup
        motor = new MotorEx(hardwareMap, "leftFlywheel", Motor.GoBILDA.BARE);
        motor.setRunMode(MotorEx.RunMode.RawPower);
        motor.setInverted(true);

        helperMotor = new MotorEx(hardwareMap, "rightFlywheel", Motor.GoBILDA.BARE);
        helperMotor.setRunMode(MotorEx.RunMode.RawPower);
        helperMotor.setInverted(false);

        now.reset();
    }

    @Override
    public void loop()
    {
        if (now.milliseconds() - inputTimeCheck >= inputTime)
        {
            input = rand.nextDouble();
            inputTime = 50 + rand.nextInt(1200); //milliseconds (min. 50, max. 1250)
            inputTimeCheck = (long) now.milliseconds();

            motor.set(input);
            helperMotor.set(input);
        }

        if (writer != null && now.milliseconds() - lastLogTime >= 50) //20 Hz
        {
            try {
                writer.write(
                    now.seconds() + "," +
                    input + "," +
                    -motor.getCorrectedVelocity() + "\n"
                );
            }
            catch (IOException e)
            {RobotLog.e("File write failed: " + e.getMessage());}
            lastLogTime = now.milliseconds();
        }

        telemetry.addData("ENCODER", motor.getCurrentPosition());
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