package org.firstinspires.ftc.teamcode.OpModes.Readers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * The purpose of this class is to track the time it takes for
 * the motor to get close to (63.2%) a commanded velocity,
 * and when and at what velocity does the motor enter steady state

 * It then outputs a table with this row structure:
 * TIMESTAMP  MOTOR INPUT (the number we put in motor.set())  ENCODER OUTPUT
 */

@TeleOp(group = "readers")
public class MotorStepReader extends OpMode
{
    private FileWriter writer;
    ElapsedTime now = new ElapsedTime();

    MotorEx motor;
    MotorEx helperMotor;

    // Steps to apply in sequence, in motor power [0,1]
    private final double[] STEPS = {0.0, 0.3, 0.0, 0.5, 0.0, 0.6, 0.0, 0.7, 0.0, 1.0, 0.0};
    private final double STEP_DURATION = 8.0; // seconds — long enough to settle

    private int stepIndex = 0;
    private double stepStartTime = 0;
    private double lastLogTime = 0;

    @Override
    public void init()
    {
        File file = new File("/sdcard/FIRST/motorData.csv");

        try {
            writer = new FileWriter(file, false);
            writer.write("timestamp,input,output\n"); // header
        }
        catch (IOException e)
        {RobotLog.e("Failed to open file: " + e.getMessage());}

        motor = new MotorEx(hardwareMap, "leftFlywheel", Motor.GoBILDA.BARE);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(true);

        helperMotor = new MotorEx(hardwareMap, "rightFlywheel", Motor.GoBILDA.BARE);
        helperMotor.setRunMode(Motor.RunMode.RawPower);
        helperMotor.setInverted(false);

        now.reset();
    }

    @Override
    public void loop()
    {
        double elapsed = now.seconds() - stepStartTime;
        if (elapsed >= STEP_DURATION)
        {
            stepIndex++;
            if (stepIndex >= STEPS.length)
            {
                motor.set(0);
                helperMotor.set(0);
                telemetry.addData("DONE", "Ran through " + STEPS.length + " steps");
                return;
            }

            stepStartTime = now.seconds();
        }

        motor.set(STEPS[stepIndex]);
        helperMotor.set(STEPS[stepIndex]);


        if (writer != null && now.seconds() - lastLogTime >= 0.05) // 20 Hz
        {
            try {
                writer.write(
                        now.seconds() + "," +
                        STEPS[stepIndex] + "," +
                        motor.getVelocity() + "\n"
                );
                writer.flush();
            }
            catch (IOException e)
            {RobotLog.e("Write failed: " + e.getMessage());}

            lastLogTime = now.seconds();
        }

        telemetry.addData("STEP", STEPS[stepIndex]);
        telemetry.addData("VELOCITY", motor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        motor.set(0);
        helperMotor.set(0);

        if (writer != null)
        {
            try { writer.flush(); writer.close(); }
            catch (IOException e) { RobotLog.e("Close failed: " + e.getMessage()); }
        }
    }
}
