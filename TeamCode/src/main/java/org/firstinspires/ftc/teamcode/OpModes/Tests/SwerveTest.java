package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(group = "tests")
public class SwerveTest extends OpMode
{
    private CRServoEx crServo;
    private AbsoluteAnalogEncoder servoAngle;
    private MotorEx motor;

    private double targetAngle = 0;
    private boolean inverted = false;


    @Override
    public void init()
    {
        servoAngle = new AbsoluteAnalogEncoder(hardwareMap, "lfAngle", 3.3, AngleUnit.RADIANS);

        crServo = new CRServoEx(hardwareMap, "lfPod", servoAngle, CRServoEx.RunMode.OptimizedPositionalControl);
        crServo.setPIDF(new PIDFCoefficients(0.2, 0.0, 0.1, 0.0001));
        crServo.setCachingTolerance(0.01);

        motor = new MotorEx(hardwareMap, "lfWheel");
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setCachingTolerance(0.01);
    }

    @Override
    public void loop()
    {
        /*if (gamepad1.right_trigger > 0.2) {crServo.setPower(gamepad1.right_trigger);}
        else {crServo.setPower(-gamepad1.left_trigger);}*/

        // Control angle of wheel using joystick, where joystick points = where wheel moves
        // (release => wheel keeps position)
        if (Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.5)
        {targetAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);}

        double currentAngle = servoAngle.getCurrentPosition();
        // Calculates the shortest path automatically
        double angleError =
                Math.atan2(
                        Math.sin(targetAngle - currentAngle),
                        Math.cos(targetAngle - currentAngle)
                );

        // Angle optimization - if error is larger than 90 degrees switch direction of motor
        // and turn to the diametrically opposite angle
        if (Math.abs(angleError) > Math.PI / 2)
        {
            targetAngle += Math.PI;
            targetAngle = wrapAngle(targetAngle);

            inverted = !inverted;
            motor.setInverted(inverted);
        }

        crServo.set(targetAngle);
        telemetry.addData("SERVO ANGLE", servoAngle.getCurrentPosition());
        telemetry.addData("TARGET ANGLE:", targetAngle);
        telemetry.addData("INVERTED", inverted);

        motor.set(-gamepad1.right_stick_y);
    }


    // Convert 0 - 360 degree range angles to -180 - 180
    public double wrapAngle(double angle)
    {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
