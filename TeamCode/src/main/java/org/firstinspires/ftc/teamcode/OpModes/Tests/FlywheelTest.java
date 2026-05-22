package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(group = "tests")
public class FlywheelTest extends OpMode
{
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;

    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR

    private double speed = 0;


    @Override
    public void init()
    {
        leftFlywheel = initMotor(hardwareMap, false, "leftFlywheel",
                /*3.25*/1, /*4.5*/0, 0);
        rightFlywheel = initMotor(hardwareMap, true, "rightFlywheel",
                /*3.25*/1, /*4.5*/0, 0);
    }

    private MotorEx initMotor(
            HardwareMap hwMap, boolean inverted, String name,
            double kp, double ki, double kd
    )
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, TICKS_PER_REV, 5800);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(kp, ki, kd);
        motor.setInverted(inverted);
        return motor;
    }

    @Override
    public void loop()
    {
        if (gamepad1.leftBumperWasPressed()) speed = 0;
        if (gamepad1.rightBumperWasPressed()) speed = 3000;
        if (gamepad1.dpadDownWasPressed()) speed -= 30;
        if (gamepad1.dpadUpWasPressed()) speed += 30;

        telemetry.addData("FLYWHEEL SPEED", speed);
        telemetry.addData("LEFT  vel (actual)", leftFlywheel.getCorrectedVelocity());
        telemetry.addData("RIGHT vel (actual)", rightFlywheel.getCorrectedVelocity());
        telemetry.addData("LEFT  pos", leftFlywheel.getCurrentPosition());
        telemetry.addData("RIGHT pos", rightFlywheel.getCurrentPosition());

        leftFlywheel.setVelocity(speed);
        rightFlywheel.setVelocity(speed);
    }
}

