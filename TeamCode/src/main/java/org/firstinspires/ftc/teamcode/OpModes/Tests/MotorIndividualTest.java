package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "tests")
public class MotorIndividualTest extends OpMode
{
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    double multiplier = 0.5;
    boolean upHeld = false;
    boolean downHeld = false;
    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotor.class, /*"flWheel"*/"intake");
        rightFront = hardwareMap.get(DcMotor.class, /*"frWheel"*/"flywheel");
        leftBack = hardwareMap.get(DcMotor.class, "blWheel");
        rightBack = hardwareMap.get(DcMotor.class, "brWheel");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop()
    {
        leftFront.setPower(gamepad1.left_trigger * multiplier);
        rightFront.setPower(gamepad1.right_trigger * multiplier);
        leftBack.setPower((gamepad1.left_bumper) ? multiplier : 0);
        rightBack.setPower((gamepad1.right_bumper) ? multiplier : 0);

        if (gamepad1.dpad_up && !upHeld)
        {
            upHeld = true;
            multiplier += 0.05;
        }
        if (!gamepad1.dpad_up) {upHeld = false;}

        if (gamepad1.dpad_down && !downHeld)
        {
            downHeld = true;
            multiplier -= 0.05;
        }
        if (!gamepad1.dpad_down) {downHeld = false;}

        telemetry.addData("Power: ",multiplier);

    }
}
