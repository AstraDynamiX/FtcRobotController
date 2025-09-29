package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoIndividualTest extends OpMode
{
    private CRServo leftGecko;
    private CRServo rightGecko;

    @Override
    public void init()
    {
        leftGecko = hardwareMap.get(CRServo.class, "leftGecko");
        rightGecko = hardwareMap.get(CRServo.class, "rightGecko");
    }

    @Override
    public void loop()
    {
        leftGecko.setPower(gamepad1.left_trigger * 0.6);
        rightGecko.setPower(gamepad1.right_trigger * 0.6);
    }
}

