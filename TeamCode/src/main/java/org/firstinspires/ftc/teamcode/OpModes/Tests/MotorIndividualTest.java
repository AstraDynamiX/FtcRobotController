package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
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
        rightFront = hardwareMap.get(DcMotor.class, "frWheel");
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
        leftFront.setPower(gamepad1.left_trigger * 0.6);
        rightFront.setPower(gamepad1.right_trigger * 0.6);
        leftBack.setPower((gamepad1.left_bumper) ? 0.6 : 0);
        rightBack.setPower((gamepad1.right_bumper) ? 0.6 : 0);
    }
}
