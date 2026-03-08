package org.firstinspires.ftc.teamcode.Mechanisms;

import static java.lang.Math.atan;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
public class LaunchBoard
{
    private final double FLYWHEEL_RADIUS = 1.89; //in
    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR
    private final double LAUNCH_HEIGHT = 13;

    public static double GOAL_HEIGHT = 41; //in; 38.19 - physical goal height
    public static double GOAL_ANGLE = 20;

    private final double MAX_LAUNCH_ANGLE = Math.toRadians(60);
    private final double MIN_LAUNCH_ANGLE = Math.toRadians(30);

    public static double TURRET_KP = 15;
    public static double FLYWHEEL_KP = 6.5;
    public static double FLYWHEEL_UNCONSTRAINTED_KP = 1;

    LimeLightBoard CamBoard = new LimeLightBoard();
    ElapsedTime stopperTimer = new ElapsedTime();

    private DcMotor intake;
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;
    private MotorEx turret;

    private Servo stopper;
    private Servo angleAdjuster;

    public DistanceSensor ballDistance;

    private LaunchState launchState = LaunchState.IDLE;

    private double launchAngle = 0;
    private double ballsShot = 0;
    private double lastFlywheelSpeed = 0;
    private double manualAdjusterAngle = 0.35;
    private double adjusterAngle = 0.3;


    public void init(HardwareMap hwMap, boolean redAlliance)
    {
        CamBoard.init(hwMap, (redAlliance) ? 7 : 8);
        //Initialize motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        leftFlywheel = initMotor(hwMap, false, "leftFlywheel",
                3.25, 4.5, 0);
        rightFlywheel = initMotor(hwMap, true, "rightFlywheel",
                3.25, 4.5, 0);

        turret = new MotorEx(hwMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setCachingTolerance(0.0001);
        turret.setPositionCoefficient(0.15);
        turret.setPositionTolerance(2);

        //stopper = new ServoEx(hwMap, "stopper", 180, AngleUnit.DEGREES);
        //angleAdjuster = new ServoEx(hwMap, "angleAdjuster", 250, AngleUnit.DEGREES);
        stopper = hwMap.get(Servo.class, "stopper");
        angleAdjuster = hwMap.get(Servo.class, "angleAdjuster");
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

    public void start() {CamBoard.start();}
    public void stop() {CamBoard.stop();}


    enum FlywheelMode
    {
        IDLE,
        AUTOMATIC,
        MANUAL
    }

    public void LaunchAdjustment(FlywheelMode mode, double flywheelMultiplier)
    {
        //Multiply this by in/s to get TPS
        double flywheelSpeed = FLYWHEEL_KP * TICKS_PER_REV / (2 * 3.1415 * FLYWHEEL_RADIUS);

        switch (mode)
        {
            // ------ Automatic launch angle and speed adjustment ------

            case AUTOMATIC:

                leftFlywheel.setVeloCoefficients(3.25, 4.5, 0);
                rightFlywheel.setVeloCoefficients(3.25, 4.5, 0);

                double distance = CamBoard.GetAprilTag("range");
                if (distance == 400) return;
                List<Double> launchAngles =
                        FindAllLaunchAngles(distance, GOAL_HEIGHT, LAUNCH_HEIGHT, Math.toRadians(GOAL_ANGLE));

                if (launchAngles.isEmpty()) return;

                double smallestLaunchSpeed = 999999;
                for (double angle : launchAngles)
                {
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
                    else {launchSpeed = LaunchSpeed(angle, distance, GOAL_ANGLE);}

                    if (launchSpeed < smallestLaunchSpeed)
                    {
                        smallestLaunchSpeed = launchSpeed;
                        launchAngle = angle;
                    }
                }

                //Recoil to compensate for temporary flywheel speed loss caused by contact with balls
                if (rightFlywheel.getVelocity() + 25 < lastFlywheelSpeed && ballsShot < 3)
                {ballsShot++;}
                lastFlywheelSpeed = rightFlywheel.getVelocity();

                //Clamp angle adjuster range
                adjusterAngle =
                        Range.scale(launchAngle, MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, 0.125, 0.9);
                        //+ 0.055 * ballsShot;

                //Divide by 2*pi*radius to get RPS then multiply by TPR to get TPS (ticks per second)
                if (smallestLaunchSpeed < 999999) {flywheelSpeed *= smallestLaunchSpeed;}

                break;

            case MANUAL:

                leftFlywheel.setVeloCoefficients(3.25, 4.5, 0);
                rightFlywheel.setVeloCoefficients(3.25, 4.5, 0);

                flywheelSpeed = flywheelMultiplier;
                adjusterAngle = manualAdjusterAngle;
                break;

            case IDLE:
            default:

                //Remove PID when slowing down because slowing down quickly is unnecessary
                //and it drains battery, since motors brake instead of letting loose
                leftFlywheel.setVeloCoefficients(0, 0, 0);
                rightFlywheel.setVeloCoefficients(0, 0, 0);
                flywheelSpeed = 0;
                break;
        }

        leftFlywheel.setVelocity(flywheelSpeed);
        rightFlywheel.setVelocity(flywheelSpeed);
        angleAdjuster.setPosition(adjusterAngle);
    }

    public void LaunchAdjustment(FlywheelMode mode) {LaunchAdjustment(mode, 1);}


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

    public void TurretMovement()
    {
        double aprilTagBearing = CamBoard.GetAprilTag("bearing");
        if (aprilTagBearing == 400)
        {
            turret.set(0);
            return;
        }

        double bearingTicks = aprilTagBearing / 360 * TICKS_PER_REV;
        double turretPosition = turret.getCurrentPosition();
        double turretTargetPosition = turretPosition - (bearingTicks * TURRET_KP);
        //Turret limits
        if (((turretPosition < -240 && turretTargetPosition < 0) ||
            (turretPosition > 450 && turretTargetPosition - turretPosition > 0))
        )
        {
            turret.setTargetPosition((int) turretPosition);
            turret.set(0);
        }
        else {turret.setTargetPosition((int) turretTargetPosition);}

        if (turret.atTargetPosition()) {turret.set(0.035);}
        else {turret.set(0.1);}
    }

    public void AngleAdjusterMovement(double input) {manualAdjusterAngle += input;}

    public double getLaunchAngle() {return launchAngle;}
    public double getTurretPosition() {return turret.getCurrentPosition();}
    public double getFlywheelSpeed() {return rightFlywheel.getVelocity();}

    // ------ Intake state machine ------

    enum LaunchState
    {
        IDLE,
        INTAKE,
        OUTTAKE,
        REV,
        SHOOT,
    }

    public void Intake()
    {
        launchState = LaunchState.INTAKE;
        ballsShot = 0;
    }

    public void Outtake() {launchState = LaunchState.OUTTAKE;}

    public void Rev() {launchState = LaunchState.REV;}

    public void Shoot()
    {
        launchState = LaunchState.SHOOT;
        stopperTimer.reset();
    }

    public void Idle() {launchState = LaunchState.IDLE;}


    public void UpdateLaunch(boolean camAdjustment, double flywheelMultiplier)
    {
        switch (launchState)
        {
            case INTAKE:

                intake.setPower(0.95);
                LaunchAdjustment(FlywheelMode.IDLE);
                stopper.setPosition(0.75);

                break;

            case OUTTAKE:

                intake.setPower(-0.8);
                LaunchAdjustment(FlywheelMode.IDLE);
                stopper.setPosition(0.75);

                break;

            case REV:

                intake.setPower(0.05);

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, flywheelMultiplier);}

                break;

            case SHOOT:

                stopper.setPosition(0.425);
                if (stopperTimer.milliseconds() < 150) return;

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, flywheelMultiplier);}

                intake.setPower(0.8);

                break;

            case IDLE:
            default:

                intake.setPower(0);
                LaunchAdjustment(FlywheelMode.IDLE);
                break;
        }
    }

    // ------ Functions that help with calculating launch angle and speed ------

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