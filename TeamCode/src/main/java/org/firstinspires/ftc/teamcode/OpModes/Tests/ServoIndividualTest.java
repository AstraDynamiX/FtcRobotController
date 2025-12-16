package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "tests")
public class ServoIndividualTest extends OpMode
{
    private Servo servo;
    private CRServo crServo;

    @Override
    public void init()
    {
        servo = hardwareMap.get(Servo.class, "transfer");
        crServo = hardwareMap.get(CRServo.class, "angleAdjuster");
    }

    @Override
    public void loop()
    {
        if (gamepad1.a) servo.setPosition(0.1);
        if (gamepad1.b) servo.setPosition(0.2);
        if (gamepad1.y) servo.setPosition(0.3);
        if (gamepad1.x) servo.setPosition(0.4);
        if (gamepad1.left_bumper) servo.setPosition(0.5);
        if (gamepad1.right_bumper) servo.setPosition(0.6);
        if (gamepad1.dpad_down) servo.setPosition(0.7);
        if (gamepad1.dpad_right) servo.setPosition(0.8);
        if (gamepad1.dpad_up) servo.setPosition(0.9);
        if (gamepad1.dpad_left) servo.setPosition(1);

        if (gamepad1.left_trigger > 0.5) {crServo.setPower(0.3);}
        else if (gamepad1.right_trigger > 0.5) {crServo.setPower(-0.3);}
        else {crServo.setPower(0);}
    }
}

