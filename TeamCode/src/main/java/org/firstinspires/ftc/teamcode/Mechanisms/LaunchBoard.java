package org.firstinspires.ftc.teamcode.Mechanisms;

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

import java.util.List;

@Configurable
public class LaunchBoard
{
    private final double GOAL_HEIGHT = 38.19; //in
    private final double FLYWHEEL_RADIUS = 1.89; //in

    CameraBoard CamBoard = new CameraBoard();

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
        CamBoard.init(hwMap);

        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        leftFlywheel = initMotor(hwMap, false, "leftFlywheel",
                0, 0, 0);
        rightFlywheel = initMotor(hwMap, true, "rightFlywheel",
                0, 0, 0);

        turret = new MotorEx(hwMap, "turret", Motor.GoBILDA.RPM_435);
        turret.setRunMode(MotorEx.RunMode.PositionControl);
        turret.setCachingTolerance(0.0001);
        turret.setPositionCoefficient(0.075);
        turret.setPositionTolerance(12);

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
        motor = new MotorEx(hwMap, name, 28, 6000);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setVeloCoefficients(kp, ki, kd);
        motor.setInverted(inverted);
        return motor;
    }

    public void IntakeMovement(double input) {intake.setPower(input);}

    public void FlywheelMovement(double input)
    {
        leftFlywheel.setVelocity(input);
        rightFlywheel.setVelocity(input);
    }

    public void TurretMovement(double input)
    {
        //turret.setVelocity(28 * 2.5 * input); //CPR * max rotations (gear ratio 3:1)
        turret.setTargetPosition((int)input);
        if (turret.atTargetPosition()) {turret.set(0);}
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
    public void Shoot() {launchState = LaunchState.SHOOTING;}
    public void Idle() {launchState = LaunchState.IDLE;}

    public double getAprilTagDistance(int id) {return CamBoard.GetAprilTag(id);}
    public double getFlywheelPower() {return flywheelPower;}
    public double getLaunchAngle() {return launchAngle;}

    public double getFlywheelVelocity() {return rightFlywheel.getVelocity();}

    public void UpdateLaunch(boolean camAdjustment, double flywheelMultiplier)
    {
        switch (launchState)
        {
            case EATING:

                intake.setPower(0.8);
                FlywheelMovement(28 * 15); //Low-power mode
                stopper.setPosition(0.75);

                /*double distance = ballDistance.getDistance(DistanceUnit.CM);

                if (distance < 5)
                {

                }*/

                break;

            case FULL:

                intake.setPower(0.05);

                if (camAdjustment)
                {
                    double distance = CamBoard.GetAprilTag(24);

                    if (distance != -1)
                    {
                        double hypotenuse = Math.sqrt(distance * distance + GOAL_HEIGHT * GOAL_HEIGHT);
                        launchAngle = Math.atan((GOAL_HEIGHT + hypotenuse) / distance);
                        flywheelPower = Math.sqrt(9.81 * (GOAL_HEIGHT + hypotenuse));
                    }
                    // 0.0873 rad = 5 deg, 0.611 rad = 35 deg
                    double adjusterAngle = 0.8 - Range.scale(launchAngle, 0.0873, 0.611, 0, 0.8);
                    angleAdjuster.setPosition(adjusterAngle * 10); // gear ratio 1/10
                    FlywheelMovement(28 * flywheelPower / (2 * 3.1415 * FLYWHEEL_RADIUS)); //Divide by that to get RPS
                }
                else {FlywheelMovement(flywheelMultiplier * 28 * 100);}

                break;

            case SHOOTING:

                if (rightFlywheel.getVelocity() + 25 < lastFlywheelSpeed && ballsShot < 3)
                {
                    angleAdjuster.setPosition(adjusterAngle + 0.075);
                    adjusterAngle += 0.075;
                    ballsShot++;
                }
                intake.setPower(0.9);
                stopper.setPosition(0.45);

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