package org.firstinspires.ftc.teamcode.Mechanisms;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class OmnimovementBoard
{
    public static double IMU_KP = 0;
    public static double IMU_KD = 0;

    MotorEx leftFrontWheel;
    MotorEx rightFrontWheel;
    MotorEx leftBackWheel;
    MotorEx rightBackWheel;
    private IMU imu;

    ElapsedTime yawLockTimer = new ElapsedTime();

    private double IMUAngle = 0;
    private double oldError = 0;

    private boolean fieldCentric = false;
    private boolean yawLockStarted = false;


    public void init(HardwareMap hwMap)
    {
        leftFrontWheel = initMotor(hwMap, true, "flWheel");
        rightFrontWheel = initMotor(hwMap, false, "frWheel");
        leftBackWheel = initMotor(hwMap, true, "blWheel");
        rightBackWheel = initMotor(hwMap, false, "brWheel");

        //Set up IMU
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parametres = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parametres);
        imu.resetYaw();
    }

    private MotorEx initMotor(HardwareMap hwMap, boolean inverted, String name)
    {
        MotorEx motor;
        motor = new MotorEx(hwMap, name, 28, 435);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setInverted(inverted);
        motor.setCachingTolerance(0.075);
        return motor;
    }

    public void ChassisMovement(double axial, double lateral, double yaw, double maxSpeed)
    {
        //Keep ratio if power exceeds motor multiplier
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), maxSpeed) / maxSpeed;

        double fedAxial = axial;
        double fedLateral = lateral;

        if (fieldCentric)
        {
            double heading = GetHeading();
            fedLateral = lateral * Math.cos(heading) - axial * Math.sin(heading);
            fedAxial = lateral * Math.sin(heading) + axial * Math.cos(heading);
        }

        double error = GetHeading() - IMUAngle;
        double P = IMU_KP * error;
        double D = IMU_KD * (error - oldError);
        //Lock movement to current angle if driver isn't yawing to prevent robot from curving when strafing
        if (yaw > -0.05 && yaw < 0.05)
        {
            if (!yawLockStarted)
            {
                IMUAngle = GetHeading();
                yawLockTimer.reset();
                yawLockStarted = true;
            }
            if (yawLockTimer.milliseconds() > 200)
            {PowerWheels(fedAxial, fedLateral, P+D, denominator);}
            else {PowerWheels(fedAxial, fedLateral, yaw, denominator);}
        }
        else
        {
            yawLockStarted = false;
            PowerWheels(fedAxial, fedLateral, yaw, denominator);
        }
        oldError = error;
        PowerWheels(axial, lateral, yaw, denominator);
    }

    public void PowerWheels(double axial, double lateral, double yaw, double denominator)
    {
        double maxTicksPerSecond = 28 * 7;
        leftFrontWheel.setVelocity(((axial - lateral - yaw) / denominator) * maxTicksPerSecond);
        leftBackWheel.setVelocity(((axial + lateral - yaw) / denominator) * maxTicksPerSecond);
        rightFrontWheel.setVelocity(((axial + lateral + yaw) / denominator) * maxTicksPerSecond);
        rightBackWheel.setVelocity(((axial - lateral + yaw) / denominator) * maxTicksPerSecond);
    }

    public void SwitchDriveMode() {fieldCentric = !fieldCentric; IMUAngle = GetHeading();}

    public double GetHeading()
    {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    public void ResetIMU() {imu.resetYaw(); IMUAngle = 0;}

}