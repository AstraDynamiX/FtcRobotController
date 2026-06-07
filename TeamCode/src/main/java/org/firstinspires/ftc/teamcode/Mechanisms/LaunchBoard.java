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
    private final double LAUNCH_HEIGHT = 13.4;

    public static double GOAL_HEIGHT = 42; //in; 38.19 - physical goal height
    public static double GOAL_ANGLE_DEGREES = 15;
    private double GOAL_ANGLE;

    private final double MAX_LAUNCH_ANGLE = Math.toRadians(60);
    private final double MIN_LAUNCH_ANGLE = Math.toRadians(30);

    public static double FLYWHEEL_KP = 11.2;
    public static double FLYWHEEL_KI = 14;

    public static double TURRET_KP = 15;
    public static double FLYWHEEL_MULTIPLIER = 1.17;
    public static double FLYWHEEL_UNCONSTRAINTED_MULTIPLIER = 1;
    public static double FLYWHEEL_BIAS = 735;

    private final double STOPPER_DOWN = 0.94;
    private final double STOPPER_UP = 0.6;


    LimeLightBoard CamBoard = new LimeLightBoard();
    ElapsedTime stopperTimer = new ElapsedTime();

    private DcMotor intake;
    private MotorEx leftFlywheel;
    private MotorEx rightFlywheel;
    private MotorEx turret;

    private Servo stopper;
    private Servo angleAdjuster;
    private Servo leftLed;
    private Servo rightLed;

    public DistanceSensor ballDistance;

    private LaunchState launchState = LaunchState.IDLE;

    private double distance = 0;
    private double launchAngle = 0;
    private double smallestLaunchSpeed = 999999;
    private double flywheelInput = 0;
    private double manualAdjusterAngle = 0.35;
    private double adjusterAngle = 0;
    private double turretCounterRotation = 0;
    private double lastFlywheelSpeed = 0;
    private double ballsShot = 0;


    public void init(HardwareMap hwMap, boolean redAlliance)
    {
        GOAL_ANGLE = Math.toRadians(GOAL_ANGLE_DEGREES);

        CamBoard.init(hwMap, (redAlliance) ? 7 : 8);
        //Initialize motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        leftFlywheel = initFlywheelMotor(hwMap, true, "leftFlywheel");
        rightFlywheel = initFlywheelMotor(hwMap, false, "rightFlywheel");

        turret = new MotorEx(hwMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setCachingTolerance(0.001);
        turret.setPositionCoefficient(0.15);
        turret.setPositionTolerance(5);

        //stopper = new ServoEx(hwMap, "stopper", 180, AngleUnit.DEGREES);
        //angleAdjuster = new ServoEx(hwMap, "angleAdjuster", 250, AngleUnit.DEGREES);
        stopper = hwMap.get(Servo.class, "stopper");
        angleAdjuster = hwMap.get(Servo.class, "angleAdjuster");

        leftLed = hwMap.get(Servo.class, "leftLed");
        rightLed = hwMap.get(Servo.class, "rightLed");

        Idle();
    }

    private MotorEx initFlywheelMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, Motor.GoBILDA.BARE);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
        motor.setInverted(inverted);
        motor.setCachingTolerance(0.01);
        return motor;
    }

    public void start() {CamBoard.start();}
    public void stop() {CamBoard.stop();}


    private enum FlywheelMode
    {
        IDLE,
        AUTOMATIC,
        MANUAL,
    }

    public void LaunchAdjustment(FlywheelMode mode, double manualFlywheelSpeed)
    {
        if (smallestLaunchSpeed == 0) flywheelInput = 0;
        // Start revving preemptively
        else if (smallestLaunchSpeed == 999999) flywheelInput = FLYWHEEL_BIAS;
        else
        {
            double launchSpeedTps = smallestLaunchSpeed * TICKS_PER_REV / (2 * 3.1415 * FLYWHEEL_RADIUS);
            lastFlywheelSpeed = flywheelInput;
            flywheelInput = (FLYWHEEL_MULTIPLIER * launchSpeedTps) + FLYWHEEL_BIAS;
        }

        leftFlywheel.setVelocity(flywheelInput);
        rightFlywheel.setVelocity(flywheelInput);
        angleAdjuster.setPosition(adjusterAngle);

        switch (mode)
        {
            // ------ Automatic launch angle and speed adjustment ------

            case AUTOMATIC:

                leftFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
                rightFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);

                distance = CamBoard.GetAprilTag("range");
                if (distance == 999999) return;

                List<Double> launchAngles = FindAllLaunchAngles(distance, GOAL_HEIGHT, LAUNCH_HEIGHT, GOAL_ANGLE);
                if (launchAngles.isEmpty()) return;

                smallestLaunchSpeed = 999999;
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

                if (lastFlywheelSpeed / flywheelInput > 1.015 && ballsShot <= 3) ballsShot++;

                //Clamp angle adjuster range
                adjusterAngle = Range.scale(launchAngle, MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE, 0.05, 0.8) + 0.02 * ballsShot;

                // Turn on LED's when flywheel is near the needed speed
                double flywheelGradient = leftFlywheel.getVelocity() / flywheelInput;
                if (flywheelGradient < 1.05 && flywheelGradient > 0.95)
                {
                    leftLed.setPosition(0.7);
                    rightLed.setPosition(0.7);
                }
                else
                {
                    leftLed.setPosition(0);
                    rightLed.setPosition(0);
                }

                break;

            case MANUAL:

                leftFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);
                rightFlywheel.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, 0);

                smallestLaunchSpeed = manualFlywheelSpeed;
                adjusterAngle = manualAdjusterAngle;
                break;

            case IDLE:
            default:

                // Remove PID when slowing down because slowing down quickly is unnecessary
                // and it drains battery, since motors brake instead of letting loose
                leftFlywheel.setVeloCoefficients(0, 0, 0);
                rightFlywheel.setVeloCoefficients(0, 0, 0);
                smallestLaunchSpeed = 0;

                leftLed.setPosition(0);
                rightLed.setPosition(0);
                break;
        }
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
        if (aprilTagBearing == 999999)
        {
            turret.set(turretCounterRotation);
            return;
        }

        double bearingTicks = aprilTagBearing / 360 * TICKS_PER_REV;
        double turretPosition = turret.getCurrentPosition();
        double turretTargetPosition = turretPosition - (bearingTicks * TURRET_KP);

        //Turret limits
        if (turretPosition < -350 && turretTargetPosition - turretPosition < 0)
        {turret.setTargetPosition((int) turretPosition + 20);}
        else if (turretPosition > 275 && turretTargetPosition - turretPosition > 0)
        {turret.setTargetPosition((int) turretPosition - 20);}

        else {turret.setTargetPosition((int) turretTargetPosition);}

        if (turret.atTargetPosition())
        {
            turret.setPositionCoefficient(0.175);
            turret.set(0.09 + turretCounterRotation);
        }
        else
        {
            turret.setPositionCoefficient(0.15);
            turret.set(0.1 + turretCounterRotation);
        }
    }

    public void AngleAdjusterMovement(double input) {manualAdjusterAngle += input;}
    public void TurretCounterRotation(double input)
    {
        /*if (Math.abs(input) > 0.2) turretCounterRotation = input;
        else turretCounterRotation = 0;*/
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.set(input);
    }

    // Getters
    public double getDistance() {return distance;}
    public double getLaunchAngle() {return Math.toDegrees(launchAngle);}
    public double getTurretPosition() {return turret.getCurrentPosition();}
    public double getFlywheelSpeed() {return flywheelInput;}

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

    public void Rev()
    {
        launchState = LaunchState.REV;
        smallestLaunchSpeed = 999999; // Start revving preemptively
    }

    public void Shoot()
    {
        launchState = LaunchState.SHOOT;
        stopperTimer.reset();
    }

    public void Idle() {launchState = LaunchState.IDLE;}


    public void UpdateLaunch(boolean camAdjustment, double manualFlywheelSpeed)
    {
        switch (launchState)
        {
            case INTAKE:

                intake.setPower(-0.95);
                LaunchAdjustment(FlywheelMode.IDLE);
                stopper.setPosition(STOPPER_DOWN);

                break;

            case OUTTAKE:

                intake.setPower(0.95);
                LaunchAdjustment(FlywheelMode.IDLE);
                stopper.setPosition(STOPPER_DOWN);

                break;

            case REV:

                intake.setPower(0.05);
                stopper.setPosition(STOPPER_DOWN);

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, manualFlywheelSpeed);}

                break;

            case SHOOT:

                stopper.setPosition(STOPPER_UP);
                // Wait for stopper to fully lift before starting transfer
                if (stopperTimer.milliseconds() < 150) return;

                intake.setPower(-0.9);

                if (camAdjustment)
                {LaunchAdjustment(FlywheelMode.AUTOMATIC);}
                else {LaunchAdjustment(FlywheelMode.MANUAL, manualFlywheelSpeed);}

                break;

            case IDLE:
            default:

                intake.setPower(0);
                stopper.setPosition(STOPPER_DOWN);
                LaunchAdjustment(FlywheelMode.IDLE);
                break;
        }
    }

    // ------ Functions that help with calculating launch angle and speed ------

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