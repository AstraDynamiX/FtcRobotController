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

@Configurable
public class LaunchBoard
{
    private final double GOAL_HEIGHT = 38.19; //in
    private final double FLYWHEEL_RADIUS = 1.89; //in
    private final double TICKS_PER_REV = 28; //Every GoBilda 5202 series motor has 28 TPR
    public static double TURRET_KP = 12;

    CameraBoard CamBoard = new CameraBoard();
    ElapsedTime stopperTimer = new ElapsedTime();
    ElapsedTime turretTimer = new ElapsedTime();
    ElapsedTime aprilTagTimer = new ElapsedTime();


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

        turret = new MotorEx(hwMap, "turret", 28, 435);
        turret.resetEncoder();
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setPositionCoefficient(0.1);
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

    public void TurretCounterYaw()
    {
        turret.setVelocity(0);
    }

    public void TurretMovement()
    {
        double aprilTagBearing = CamBoard.GetAprilTag(24, "bearing");
        double bearingTicks = -aprilTagBearing / 360 * TICKS_PER_REV * 3  * TURRET_KP; //gear ratio 3/1
        turret.setTargetPosition((int) (turret.getCurrentPosition() + bearingTicks));

        if (turret.atTargetPosition()) {turret.set(0);}
        else {turret.set(0.125);}

        /*if (aprilTagBearing == 400 /*&& aprilTagTimer.milliseconds() > 400 ) {
                turret.setVelocity(0);
              if (turretTimer.milliseconds() < 1500) {turret.setVelocity(-41);}
              else {turret.setVelocity(41);}
            if (turretTimer.milliseconds()>3000){turretTimer.reset();}
              aprilTagTimer.reset();
        }
        else {
            error = (aprilTagBearing / 360);
            P = error * TURRETKP;
            D = TURRETKD * (error - errorOld);
            integral += error;
            I = TURRETKI * integral;
            turret.setVelocity((int)((P + D + I) * TICKS_PER_REV * 3));//gear ratio 3/1
            errorOld = error;
            turretTimer.reset();}*/
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

    public void Intake() //Temporary until automatic detection1
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
    public double getBearingTicks()
    {
        double aprilTagBearing = CamBoard.GetAprilTag(24, "bearing");
        double bearingTicks = -aprilTagBearing / 360 * TICKS_PER_REV * 3 * TURRET_KP;
        return bearingTicks;
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
                    double adjusterAngle = 0.8 - Range.scale(launchAngle, 0.0873, 0.611, 0, 0.8);
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

}