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
    private Servo servo1;
    private Servo servo2;
    private CRServoEx crServo;
    private AbsoluteAnalogEncoder crServoAngle;

    private double servoPos1 = 0.5;
    private double servoPos2 = 0.5;


    @Override
    public void init()
    {
        servo1 = hardwareMap.get(Servo.class, "angleAdjuster");
        servo2 = hardwareMap.get(Servo.class, "stopper");
        /*crServoAngle = new AbsoluteAnalogEncoder(hardwareMap, "lfAngle", 3.3, AngleUnit.RADIANS);
        crServo = new CRServoEx(hardwareMap, "lfPod");
        crServo.setRunMode(CRServoEx.RunMode.RawPower);
        crServo.setCachingTolerance(0.01);*/
    }

    @Override
    public void loop()
    {
        if (gamepad1.leftBumperWasPressed()) servoPos1 = 0;
        if (gamepad1.rightBumperWasPressed()) servoPos1 = 1;
        if (gamepad1.dpadDownWasPressed()) servoPos1 -= 0.05;
        if (gamepad1.dpadUpWasPressed()) servoPos1 += 0.05;

        telemetry.addData("SERVO 1", servoPos1);
        servo1.setPosition(servoPos1);

        if (gamepad1.left_trigger > 0.2) servoPos2 = 0;
        if (gamepad1.right_trigger > 0.2) servoPos2 = 1;
        if (gamepad1.dpadLeftWasPressed()) servoPos2 -= 0.02;
        if (gamepad1.dpadRightWasPressed()) servoPos2 += 0.02;

        telemetry.addData("SERVO 2", servoPos2);
        servo2.setPosition(servoPos2);

        /*if (gamepad1.right_trigger > 0.2) {crServo.set(gamepad1.right_trigger);}
        else {crServo.set(-gamepad1.left_trigger);}

        telemetry.addData("CR SERVO POSITION", Math.toDegrees(crServoAngle.getCurrentPosition()));*/
    }
}

