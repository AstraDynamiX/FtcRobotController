package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Mechanisms.LimeLightBoard;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(group = "tests")
public class AutomaticAdjustmentTest extends OpMode
{
    public static double GOAL_HEIGHT = 42; //in; 38.19 - physical goal height
    private final double LAUNCH_HEIGHT = 13.4;
    public static double GOAL_ANGLE_DEGREES = 15;
    private double GOAL_ANGLE;

    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR
    private final double FLYWHEEL_RADIUS = 1.89; //in
    public static double FLYWHEEL_KP = 11.2;
    public static double FLYWHEEL_KI = 14;

    public static double FLYWHEEL_MULTIPLIER = 1.17;
    public static double FLYWHEEL_UNCONSTRAINTED_MULTIPLIER = 1;
    public static double FLYWHEEL_BIAS = 735; // TPS

    private final double MAX_LAUNCH_ANGLE = Math.toRadians(60);
    private final double MIN_LAUNCH_ANGLE = Math.toRadians(30);


    LimeLightBoard CamBoard = new LimeLightBoard();
    private FileWriter writer;

    private Servo angleAdjuster;
    private DcMotor intake;
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;
    private MotorEx turret;

    private double launchAngle = 0;
    private double smallestLaunchSpeed = 999999;
    private boolean motorControl = false;
    private boolean servoControl = false;
    private double distance = 0;


    @Override
    public void init()
    {
        CamBoard.init(hardwareMap, 8);

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFlywheel = initMotor(hardwareMap, true, "leftFlywheel");
        rightFlywheel = initMotor(hardwareMap, false, "rightFlywheel");
        angleAdjuster = hardwareMap.get(Servo.class, "angleAdjuster");

        turret = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setCachingTolerance(0.001);
        turret.setPositionCoefficient(0.15);
        turret.setPositionTolerance(5);

        File file = new File("/sdcard/FIRST/autoAdjustmentData.csv");

        try {
            writer = new FileWriter(file, false);
            writer.write("distance,launchAngle,launchSpeed\n"); // header
        }
        catch (IOException e)
        {RobotLog.e("Failed to open file: " + e.getMessage());}
    }

    private MotorEx initMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, Motor.GoBILDA.BARE);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
        motor.setInverted(inverted);
        return motor;
    }

    @Override
    public void start() {CamBoard.start();}

    @Override
    public void loop()
    {
        GOAL_ANGLE = Math.toRadians(GOAL_ANGLE_DEGREES);

        distance = CamBoard.GetAprilTag("range");
        if (distance == 999999) return;

        List<Double> launchAngles = FindAllLaunchAngles(distance, GOAL_HEIGHT, LAUNCH_HEIGHT, GOAL_ANGLE);
        StringBuilder anglesFound = new StringBuilder();

        if (launchAngles.isEmpty()) return;

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
                    angle = MAX_LAUNCH_ANGLE;
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

        //Divide in/s by 2*pi*radius to get RPS then multiply by TPR to get TPS (ticks per second)
        double launchSpeedTps = smallestLaunchSpeed * TICKS_PER_REV / (2 * 3.1415 * FLYWHEEL_RADIUS);
        double flywheelInput = (FLYWHEEL_MULTIPLIER * launchSpeedTps) + FLYWHEEL_BIAS;

        telemetry.addData("DISTANCE", distance);
        telemetry.addData("ANGLES FOUND", anglesFound);
        telemetry.addData("LAUNCH ANGLE", Math.toDegrees(launchAngle));
        telemetry.addData("LAUNCH SPEED", flywheelInput);
        telemetry.addData("RAW LAUNCH SPEED", launchSpeedTps);

        if (gamepad1.xWasPressed() && writer != null)
        {
            try {
                writer.write(
                        distance + "," +
                        Math.toDegrees(launchAngle) + "," +
                        flywheelInput + "\n"
                );
                writer.flush();
            }
            catch (IOException e)
            {RobotLog.e("Write failed: " + e.getMessage());}
        }

        if (gamepad1.aWasPressed())
        {motorControl = !motorControl;}

        if (gamepad1.bWasPressed())
        {servoControl = !servoControl;}

        if (motorControl)
        {
            //Divide by 2*pi*radius to get RPS then multiply by TPR to get TPS (ticks per second)
            leftFlywheel.setVelocity(flywheelInput);
            rightFlywheel.setVelocity(flywheelInput);

            intake.setPower(-0.8);
        }
        else
        {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
            intake.setPower(0);
        }

        if (servoControl)
        {
            //Clamp angle adjuster range
            double adjusterAngle = Range.scale(launchAngle, MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, 0.05, 0.8);
            angleAdjuster.setPosition(adjusterAngle);
        }

        TurretLockPosition(0);
    }

    @Override
    public void stop()
    {
        CamBoard.stop();

        if (writer != null)
        {
            try { writer.flush(); writer.close(); }
            catch (IOException e) { RobotLog.e("Close failed: " + e.getMessage()); }
        }
    }


    public void TurretLockPosition(double input)
    {
        turret.setTargetPosition((int)input);
        if (turret.atTargetPosition())
        {
            turret.setPositionCoefficient(0.175);
            turret.set(0.05);
        }
        else
        {
            turret.setPositionCoefficient(0.15);
            turret.set(0.1);
        }
    }

    private double LaunchSpeed(double launchAngle, double x, double goalAngle)
    {
        //gravitational acceleration in in/s^2 = 386.09
        double g = 386.09;
        return Math.sqrt(
                (g * x * Math.cos(launchAngle) * Math.cos(goalAngle)) /
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

        return Math.sqrt(g * x * x / denom);
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
