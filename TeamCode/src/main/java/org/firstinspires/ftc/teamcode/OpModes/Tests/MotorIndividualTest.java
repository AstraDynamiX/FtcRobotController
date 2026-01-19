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

    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotor.class, "flWheel");
        rightFront = hardwareMap.get(DcMotor.class, "rightFlywheel");
        leftBack = hardwareMap.get(DcMotor.class, "leftFlywheel");
        rightBack = hardwareMap.get(DcMotor.class, "turret");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop()
    {
        leftFront.setPower(gamepad1.left_trigger * 0.6);
        rightFront.setPower(gamepad1.right_trigger * 0.6);
        leftBack.setPower((gamepad1.left_bumper) ? 0.6 : 0);
        rightBack.setPower((gamepad1.right_bumper) ? 0.6 : 0);

        telemetry.addData("ENCODER", leftFront.getCurrentPosition());
        telemetry.addData("ENCODER", rightFront.getCurrentPosition());
        telemetry.addData("ENCODER", leftBack.getCurrentPosition());
        telemetry.addData("ENCODER", rightBack.getCurrentPosition());
    }
}
