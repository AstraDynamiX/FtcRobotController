package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import java.util.Arrays;


public class SwerveBoard
{
    private final double TICKS_PER_REV = 28; // TPR for GoBilda 5202 series motor
    private final double RPM = 5800; // Tested RPM for GoBilda 5202 series motor
    private final double NOISE_FILTER_COEF = 0.5; // 0 = angle doesn't change, 1 = no filtering

    private final double[] WHEEL_DISTANCE_X = {0, 0, 0, 0};
    private final double[] WHEEL_DISTANCE_Y = {0, 0, 0, 0};

    // Pods are initialized in this order in the lists: left front, right front, left back, right back
    private final String[] POSITION_NAMES = {"lf", "rf", "lb", "rb"};

    MotorEx[] motors;
    CRServoEx[] servos;
    private AbsoluteAnalogEncoder[] servoEncoders;

    private double[] wheelSpeeds;
    private double fCurrentWheelAngle = 0;


    public void init(HardwareMap hwMap)
    {
        for (int i = 0; i < 4; i++)
        {
            motors[i] = initMotor(hwMap, false, Arrays.toString(POSITION_NAMES) + "Motor");
            servos[i] = initServo(hwMap, Arrays.toString(POSITION_NAMES) + "Servo", i);
        }
    }

    private MotorEx initMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor = new MotorEx(hwMap, name, TICKS_PER_REV, RPM);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(0, 0, 0);
        motor.setInverted(inverted);
        motor.setCachingTolerance(0.01);
        return motor;
    }

    private CRServoEx initServo(HardwareMap hwMap, String name, int number)
    {
        servoEncoders[number] = new AbsoluteAnalogEncoder(hwMap, name + "Input");
        CRServoEx servo = new CRServoEx(hwMap, name, servoEncoders[number], CRServoEx.RunMode.OptimizedPositionalControl);
        servo.setPIDF(new PIDFCoefficients(0, 0, 0, 0)); //TODO
        return servo;
    }

    // cmdV = commanded velocity
    public void ChassisMovement(double cmdVX, double cmdVY, double cmdVRot, double maxSpeed)
    {
        for (int i = 0; i < 4; i++)
        {
            // Calculate individual wheel velocities on axes x and y
            double wheelVX = cmdVX - cmdVRot * WHEEL_DISTANCE_Y[i];
            double wheelVY = cmdVY + cmdVRot * WHEEL_DISTANCE_X[i];
            // Calculate the needed speed and angle of the wheel in order to achieve said velocities
            double wheelSpeed = Math.sqrt(wheelVX*wheelVX + wheelVY*wheelVY);
            double wheelAngle = Math.atan2(wheelVX, wheelVY);

            double currentWheelAngle = servoEncoders[i].getCurrentPosition();
            // Simple exponential filtering to reduce noise (f = filtered)
            fCurrentWheelAngle = NOISE_FILTER_COEF * currentWheelAngle + (1 - NOISE_FILTER_COEF) * fCurrentWheelAngle;
            // Cosine compensation - reduce wheel speed based on angle error to reduce
            wheelSpeed *= Math.cos(wheelAngle - fCurrentWheelAngle);

            wheelSpeeds[i] = wheelSpeed;
            servos[i].set(Math.toRadians(wheelAngle));
        }

        // Scale down all wheel speeds proportionally if a speed surpasses the max speed
        double denominator = maxSpeed;
        for (int i = 0; i < 4; i++)
        {
            if (wheelSpeeds[i] > denominator)
            {denominator = wheelSpeeds[i];}
        }

        for (int i = 0; i < 4; i++)
        {
            motors[i].setVelocity(wheelSpeeds[i] / denominator * RPM * TICKS_PER_REV);
        }
    }

}
