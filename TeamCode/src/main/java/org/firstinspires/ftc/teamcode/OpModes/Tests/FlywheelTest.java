package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(group = "tests")
public class FlywheelTest extends OpMode
{
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;

    private double speed = 0;


    @Override
    public void init()
    {
        leftFlywheel = initMotor(hardwareMap, true, "leftFlywheel",
                11.2, 14, 0);
        rightFlywheel = initMotor(hardwareMap, false, "rightFlywheel",
                11.2, 14, 0);
    }

    private MotorEx initMotor(
            HardwareMap hwMap, boolean inverted, String name,
            double kp, double ki, double kd
    )
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, Motor.GoBILDA.BARE);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(kp, ki, kd);
        motor.setInverted(inverted);
        return motor;
    }

    @Override
    public void loop()
    {
        if (gamepad1.leftBumperWasPressed()) speed = 0;
        if (gamepad1.rightBumperWasPressed()) speed = 1400;
        if (gamepad1.dpadDownWasPressed()) speed -= 20;
        if (gamepad1.dpadUpWasPressed()) speed += 20;

        telemetry.addData("TARGET SPEED", speed);
        telemetry.addData("LEFT  FLYWHEEL SPEED", leftFlywheel.getVelocity());
        telemetry.addData("RIGHT FLYWHEEL SPEED", rightFlywheel.getVelocity());

        leftFlywheel.setVelocity(speed);
        rightFlywheel.setVelocity(speed);
    }
}

