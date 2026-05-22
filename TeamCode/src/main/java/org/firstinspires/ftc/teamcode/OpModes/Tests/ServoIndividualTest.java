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

    private double servoPos = 0.5;

    @Override
    public void init()
    {
        servo = hardwareMap.get(Servo.class, "leftLed");
        crServo = hardwareMap.get(CRServo.class, "lfPod");
    }

    @Override
    public void loop()
    {
        if (gamepad1.leftBumperWasPressed()) servoPos = 0;
        if (gamepad1.rightBumperWasPressed()) servoPos = 1;
        if (gamepad1.dpadDownWasPressed()) servoPos -= 0.05;
        if (gamepad1.dpadUpWasPressed()) servoPos += 0.05;

        telemetry.addData("SERVO POSITION", servoPos);

        servo.setPosition(servoPos);

        if (gamepad1.right_trigger > 0.2) {crServo.setPower(gamepad1.right_trigger);}
        else {crServo.setPower(-gamepad1.left_trigger);}
    }
}

