package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "tests")
public class ServoIndividualTest extends OpMode
{
    private Servo servo;
    private CRServoEx crServo;
    private AbsoluteAnalogEncoder crServoAngle;

    private double servoPos = 0.5;


    @Override
    public void init()
    {
        servo = hardwareMap.get(Servo.class, "angleAdjuster");
        /*crServoAngle = new AbsoluteAnalogEncoder(hardwareMap, "lfAngle", 3.3, AngleUnit.RADIANS);
        crServo = new CRServoEx(hardwareMap, "lfPod");
        crServo.setRunMode(CRServoEx.RunMode.RawPower);
        crServo.setCachingTolerance(0.01);*/
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

        /*if (gamepad1.right_trigger > 0.2) {crServo.set(gamepad1.right_trigger);}
        else {crServo.set(-gamepad1.left_trigger);}

        telemetry.addData("CR SERVO POSITION", Math.toDegrees(crServoAngle.getCurrentPosition()));*/
    }
}

