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

@Configurable
public class LaunchBoard
{
    private final double GOAL_HEIGHT = 38.19; //in
    private final double FLYWHEEL_RADIUS = 1.89; //in
    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR

    CameraBoard CamBoard = new CameraBoard();
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
    private double flywheelPower = 0;
    private double ballsShot = 0;
    private double lastFlywheelSpeed = 0;
    private double adjusterAngle = 0.35;


    public void init(HardwareMap hwMap)
    {
        //Initialize motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        leftFlywheel = initMotor(hwMap, false, "leftFlywheel",
                3.25, 4.5, 0);
        rightFlywheel = initMotor(hwMap, true, "rightFlywheel",
                3.25, 4.5, 0);

        turret = new MotorEx(hwMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setCachingTolerance(0.0001);
        turret.setPositionCoefficient(0.09);
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

    public void initCamera(HardwareMap hwMap) {CamBoard.init(hwMap);}

    public void IntakeMovement(double input) {intake.setPower(input);}

    public void FlywheelMovement(double input)
    {
        leftFlywheel.setVelocity(input);
        rightFlywheel.setVelocity(input);
    }

    public void TurretLockPosition(double input)
    {
        turret.setTargetPosition((int)input);
        if (turret.atTargetPosition()) {turret.set(0.035);}
        else {turret.set(0.1);}
    }

    public void TurretMovement()
    {
        double[] aprilTagDimensions = CamBoard.GetAprilTag(24);
        double aprilTagDistance = aprilTagDimensions[0];
        double aprilTagBearing = aprilTagDimensions[1];

        if (aprilTagDistance == 0 || aprilTagBearing == 0) {return;}
        turret.setTargetPosition((int)(TICKS_PER_REV * atan(aprilTagDistance/aprilTagBearing) / 360 * 10)); //gear ratio 10/1
        if (turret.atTargetPosition()) {turret.set(0.05);}
        else {turret.set(0.1);}
    }

    public void AngleAdjusterMovement(double input) {adjusterAngle += input;}
    public void UpdateAngleAdjuster() {angleAdjuster.setPosition(adjusterAngle);}
    public double getAdjusterAngle() {return adjusterAngle;}

    // --- Intake state machine ---
    enum LaunchState
    {
        IDLE,
        EATING,
        FULL,
        SHOOTING,
    }

    public void Intake() //Temporary until automatic detection
    {
        launchState = LaunchState.EATING;
        adjusterAngle -= 0.75 * ballsShot;
        ballsShot = 0;
    }
    public void Rev() {launchState = LaunchState.FULL;} //Temporary until automatic detection
    public void Shoot()
    {
        launchState = LaunchState.SHOOTING;
        stopperTimer.reset();
    }
    public void Idle() {launchState = LaunchState.IDLE;}

    public double getAprilTagDistance(int id) {return CamBoard.GetAprilTag(id, "range");}
    public double getAprilTagBearing(int id) {return CamBoard.GetAprilTag(id, "bearing");}
    public double getFlywheelPower() {return flywheelPower;}
    public double getLaunchAngle() {return launchAngle;}

    public double getTurretPosition() {return turret.getCurrentPosition();}
    public double getTurretInput()
    {
        double[] aprilTagDimensions = CamBoard.GetAprilTag(24);
        double aprilTagDistance = aprilTagDimensions[0];
        double aprilTagBearing = aprilTagDimensions[1];

        if (aprilTagDistance == 0 || aprilTagBearing == 0) {return 0;}
        return (TICKS_PER_REV * atan(aprilTagDistance/aprilTagBearing) / 360 * 10); //gear ratio 10/1
    }

    public void UpdateLaunch(boolean camAdjustment, double flywheelMultiplier)
    {
        switch (launchState)
        {
            case EATING:

                intake.setPower(0.8);
                FlywheelMovement(TICKS_PER_REV * 15); //Low-power mode
                stopper.setPosition(0.9);

                /*double distance = ballDistance.getDistance(DistanceUnit.CM);

                if (distance < 5)
                {

                }*/

                break;

            case FULL:

                intake.setPower(0.05);

                if (camAdjustment)
                {
                    double distance = CamBoard.GetAprilTag(24, "range");

                    if (distance != -1)
                    {
                        double hypotenuse = Math.sqrt(distance * distance + GOAL_HEIGHT * GOAL_HEIGHT);
                        launchAngle = atan((GOAL_HEIGHT + hypotenuse) / distance);
                        flywheelPower = Math.sqrt(386.09 * (GOAL_HEIGHT + hypotenuse)); //gravitational acceleration in in/s^2
                    }
                    // 0.0873 rad = 5 deg, 0.611 rad = 35 deg
                    double adjusterAngle = 0.9 - Range.scale(launchAngle, 0.0873, 0.611, 0.1, 0.9);
                    angleAdjuster.setPosition(adjusterAngle);
                    FlywheelMovement(TICKS_PER_REV * flywheelPower / (2 * 3.1415 * FLYWHEEL_RADIUS)); //Divide by that to get RPS
                }
                else {FlywheelMovement(flywheelMultiplier * TICKS_PER_REV * 100);}

                break;

            case SHOOTING:

                stopper.setPosition(0.45);
                if (stopperTimer.milliseconds() < 150) {return;}

                if (rightFlywheel.getVelocity() + 25 < lastFlywheelSpeed && ballsShot < 3)
                {
                    angleAdjuster.setPosition(adjusterAngle + 0.075);
                    adjusterAngle += 0.075;
                    ballsShot++;
                }
                intake.setPower(0.9);

                lastFlywheelSpeed = rightFlywheel.getVelocity();

                break;

            case IDLE:
            default:

                intake.setPower(0);
                FlywheelMovement(0);
                break;
        }
    }

    // ------ Functions that help with calculating launch angle and speed ------

    private double LaunchSpeed(double launchAngle, double x, double alpha)
    {
        return Math.sqrt(
                (9.81 * x * Math.cos(launchAngle) * Math.cos(alpha)) /
                        Math.sin(launchAngle + alpha)
        );
    }

    //Separate function into sections and find sections in which function crosses 0
    //to find all roots, because function below only converges to one of potential two solutions
    List<Double> FindAllLaunchAngles(
            double x, double y, double h, double alpha
    ) {
        List<Double> roots = new ArrayList<>();

        double start = Math.toRadians(50.0);
        double end = Math.toRadians(85.0);
        int sections = 100;

        double section = (end - start) / sections;

        double sectionStart = start;
        double fSectionStart = LaunchAngleFunction(sectionStart, x, y, h, alpha);

        for (int i = 1; i <= sections; i++)
        {
            double sectionEnd = start + i * section;
            double fSectionEnd = LaunchAngleFunction(sectionEnd, x, y, h, alpha);

            if (fSectionStart * fSectionEnd <= 0) //Function crosses 0, so a root is there
            {
                Double root = SearchForLaunchAngle(x, y, h, alpha, sectionStart, sectionEnd);
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
            double x, double y, double h, double alpha,
            double launchAngleMin, double launchAngleMax
    ) {
        double fLaunchAngleMin = LaunchAngleFunction(launchAngleMin, x, y, h, alpha);
        double fLaunchAngleMax = LaunchAngleFunction(launchAngleMax, x, y, h, alpha);

        //Same sign, so top assumption is false, so there's no solution
        if (fLaunchAngleMin * fLaunchAngleMax > 0) {return null;}

        for (int i = 0; i < 50; i++) //~1e-15 precision
        {
            double launchAngleMid = 0.5 * (launchAngleMin + launchAngleMax);
            double fLaunchAngleMid = LaunchAngleFunction(launchAngleMid, x, y, h, alpha);

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
    private double LaunchAngleFunction(double launchAngle, double x, double y, double h, double alpha)
    {
        return x * Math.tan(launchAngle)
                - (x * Math.sin(launchAngle + alpha)) / (2.0 * Math.cos(launchAngle) * Math.cos(alpha))
                - (y - h);
    }

}