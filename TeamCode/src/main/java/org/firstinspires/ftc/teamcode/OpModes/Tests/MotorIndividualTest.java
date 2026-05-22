package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@TeleOp(group = "tests")
public class MotorIndividualTest extends OpMode
{
    private final String[] MOTOR_NAMES = {"leftFlywheel", "rightFlywheel"}; // Names used in Control Hub config
    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR

    private MotorEx[] motors = new MotorEx[MOTOR_NAMES.length];
    // Automatically updates values based on current inputs
    private final Supplier<Float>[] inputs = new Supplier[] {
            () -> gamepad1.left_stick_y,
            () -> gamepad1.right_stick_y,
            () -> gamepad1.left_trigger,
            () -> gamepad1.right_trigger
    };

    @Override
    public void init()
    {
        for (int i = 0; i < MOTOR_NAMES.length; i++)
        {
            // Check if name exists in config
            if (hardwareMap.get(MOTOR_NAMES[i]) != null)
            {
                motors[i] = initMotor(hardwareMap, MOTOR_NAMES[i]);
                motors[i].resetEncoder();
            }
        }
    }

    private MotorEx initMotor(HardwareMap hwMap, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, TICKS_PER_REV, 5800);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        return motor;
    }

    @Override
    public void loop()
    {
        for (int i = 0; i < motors.length; i++)
        {
            if (motors[i] != null)
            {
                motors[i].setVelocity(inputs[i].get() * 0.9 * TICKS_PER_REV * 5800);
                telemetry.addData(MOTOR_NAMES[i], motors[i].getCurrentPosition());
            }
        }
    }

}
