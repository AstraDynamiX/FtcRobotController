package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Mechanisms.LimeLightBoard;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
@Autonomous(group = "tests")
public class AutomaticAdjustmentTest extends OpMode
{
    public static double GOAL_HEIGHT = 42; //in; 38.19 - physical goal height
    private final double LAUNCH_HEIGHT = 13;
    public static double GOAL_ANGLE = 35;

    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR
    private final double FLYWHEEL_RADIUS = 1.89; //in
    public static double FLYWHEEL_KP = 5.5;
    public static double FLYWHEEL_UNCONSTRAINTED_KP = 1.8;
    private static double FAR_ANGLE = 10;

    private final double MAX_LAUNCH_ANGLE = Math.toRadians(60);
    private final double MIN_LAUNCH_ANGLE = Math.toRadians(30);

    LimeLightBoard CamBoard = new LimeLightBoard();

    private Servo angleAdjuster;
    private DcMotor intake;
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;

    double launchAngle = 0;
    double smallestLaunchSpeed = 999999;
    boolean motorControl = false;
    boolean aHeld = false;
    double distance = 0;


    @Override
    public void init()
    {
        CamBoard.init(hardwareMap, 8);

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFlywheel = initMotor(hardwareMap, false, "leftFlywheel",
                3.25, 4.5, 0);
        rightFlywheel = initMotor(hardwareMap, true, "rightFlywheel",
                3.25, 4.5, 0);
        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");

        GOAL_ANGLE = Math.toRadians(GOAL_ANGLE);
    }

    private MotorEx initMotor(
            HardwareMap hwMap, boolean inverted, String name,
            double kp, double ki, double kd
    )
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, TICKS_PER_REV, 6000);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(kp, ki, kd);
        motor.setInverted(inverted);
        return motor;
    }

    @Override
    public void start() {CamBoard.start();}

    @Override
    public void loop()
    {
        double newDistance = CamBoard.GetAprilTag("range");
        if (newDistance != 400) {distance = newDistance;}
        List<Double> launchAngles = FindAllLaunchAngles(distance, GOAL_HEIGHT, LAUNCH_HEIGHT, GOAL_ANGLE);
        StringBuilder anglesFound = new StringBuilder();

        if (launchAngles.isEmpty()) {return;}

        smallestLaunchSpeed = 999999;
        for (double angle : launchAngles)
        {
            anglesFound.append(String.format(Locale.US, "{%.3f}", Math.toDegrees(angle)));
            anglesFound.append("; ");

            double launchSpeed;
            if (angle < MIN_LAUNCH_ANGLE || angle > MAX_LAUNCH_ANGLE)
            {
                double minAngleSpeed = UnconstrainedLaunchSpeed(MIN_LAUNCH_ANGLE, distance, GOAL_HEIGHT, LAUNCH_HEIGHT);
                double maxAngleSpeed = UnconstrainedLaunchSpeed(MAX_LAUNCH_ANGLE, distance, GOAL_HEIGHT, LAUNCH_HEIGHT);

                if (minAngleSpeed < maxAngleSpeed)
                {
                    angle = MIN_LAUNCH_ANGLE;
                    launchSpeed = minAngleSpeed;
                }
                else
                {
                    angle = MAX_LAUNCH_ANGLE - Math.toRadians(FAR_ANGLE); //Hardware issues call for hardcoded magic
                    launchSpeed = maxAngleSpeed;
                }
            }
            else
            {launchSpeed = LaunchSpeed(angle, distance, GOAL_ANGLE);}


            if (launchSpeed < smallestLaunchSpeed)
            {
                smallestLaunchSpeed = launchSpeed;
                launchAngle = angle;
            }
        }

        telemetry.addData("ANGLES FOUND", anglesFound);
        telemetry.addData("LAUNCH SPEED", smallestLaunchSpeed);

        if (motorControl)
        {
            //Clamp angle adjuster range
            double adjusterAngle =
                    Range.scale(launchAngle, MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, 0.125, 0.9);
            angleAdjuster.setPosition(adjusterAngle);

            //Divide by 2*pi*radius to get RPS then multiply by TPR to get TPS (ticks per second)
            double flywheelInput = FLYWHEEL_KP * TICKS_PER_REV * smallestLaunchSpeed / (2 * 3.1415 * FLYWHEEL_RADIUS);
            leftFlywheel.setVelocity(flywheelInput);
            rightFlywheel.setVelocity(flywheelInput);
            telemetry.addData("FLYWHEEL SPEED", flywheelInput);

            intake.setPower(0.8);
        }
        else
        {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
            intake.setPower(0);
        }

        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            motorControl = !motorControl;
        }
        if (!gamepad1.a) {aHeld = false;}
    }

    @Override
    public void stop() {CamBoard.stop();}


    private double LaunchSpeed(double launchAngle, double x, double goalAngle)
    {
        //gravitational acceleration in in/s^2 = 386.09
        return Math.sqrt(
                (386.09 * x * Math.cos(launchAngle) * Math.cos(goalAngle)) /
                        Math.sin(launchAngle + goalAngle)
        );
    }

    //For angles outside of the bounds of the angle adjuster, we compensate with speed
    private double UnconstrainedLaunchSpeed(double launchAngle, double x, double y, double h)
    {
        double g = 386.09;

        double denom = 2.0 * Math.pow(Math.cos(launchAngle), 2) *
                (x * Math.tan(launchAngle) - (y - h));

        if (denom <= 0) return 999999; //Impossible at this angle

        return Math.sqrt(g * x * x / denom) * FLYWHEEL_UNCONSTRAINTED_KP;
    }

    //Separate function into sections and find sections in which function crosses 0
    //to find all roots, because function below only converges to one of potential two solutions
    List<Double> FindAllLaunchAngles(
            double x, double y, double h, double goalAngle
    ) {
        List<Double> roots = new ArrayList<>();

        double start = Math.toRadians(5);
        double end = Math.toRadians(85);
        int sections = 100;

        double section = (end - start) / sections;

        double sectionStart = start;
        double fSectionStart = LaunchAngleFunction(sectionStart, x, y, h, goalAngle);

        for (int i = 1; i <= sections; i++)
        {
            double sectionEnd = start + i * section;
            double fSectionEnd = LaunchAngleFunction(sectionEnd, x, y, h, goalAngle);

            if (fSectionStart * fSectionEnd <= 0) //Function crosses 0, so a root is there
            {
                Double root = SearchForLaunchAngle(x, y, h, goalAngle, sectionStart, sectionEnd);
                if (root != null) {roots.add(root);}
            }

            sectionStart = sectionEnd;
            fSectionStart = fSectionEnd;
        }

        return roots;
    }

    //Bisection, if one angle overshoots and another undershoots then the
    //correct angle is somewhere in the middle
    private Double SearchForLaunchAngle(
            double x, double y, double h, double goalAngle,
            double launchAngleMin, double launchAngleMax
    ) {
        double fLaunchAngleMin = LaunchAngleFunction(launchAngleMin, x, y, h, goalAngle);
        double fLaunchAngleMax = LaunchAngleFunction(launchAngleMax, x, y, h, goalAngle);

        //Same sign, so top assumption is false, so there's no solution
        if (fLaunchAngleMin * fLaunchAngleMax > 0) {return null;}

        for (int i = 0; i < 50; i++) //~1e-15 precision
        {
            double launchAngleMid = 0.5 * (launchAngleMin + launchAngleMax);
            double fLaunchAngleMid = LaunchAngleFunction(launchAngleMid, x, y, h, goalAngle);

            //Opposite signs, so correct angle is in between
            if (fLaunchAngleMin * fLaunchAngleMid <= 0)
            {
                launchAngleMax = launchAngleMid;
                fLaunchAngleMax = fLaunchAngleMid;
            } else {
                launchAngleMin = launchAngleMid;
                fLaunchAngleMin = fLaunchAngleMid;
            }
        }
        //Practically converges to one value because the difference becomes insignificant
        return 0.5 * (launchAngleMin + launchAngleMax);
    }

    //The closer the value this returns is to 0, the closer to the correct launch angle
    //(check technical book for how we got to this equation)
    private double LaunchAngleFunction(double launchAngle, double x, double y, double h, double goalAngle)
    {
        return x * Math.tan(launchAngle)
                - (x * Math.sin(launchAngle + goalAngle)) / (2.0 * Math.cos(launchAngle) * Math.cos(goalAngle))
                - (y - h);
    }
}
