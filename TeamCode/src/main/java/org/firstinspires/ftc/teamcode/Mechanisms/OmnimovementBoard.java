package org.firstinspires.ftc.teamcode.Mechanisms;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class OmnimovementBoard
{
    public static double IMU_KP = 3;
    public static double IMU_KD = 5;

    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftBackWheel;
    private DcMotor rightBackWheel;
    private IMU imu;

    //State machine
    OmnimovementBoard.YawLockState yawLockState = OmnimovementBoard.YawLockState.IDLE;
    ElapsedTime yawLockTimer = new ElapsedTime();

    private double IMUAngle = 0;
    private double oldError = 0;

    private boolean fieldCentric = false;
    private boolean yawLockStarted = false;
    private boolean yawLocked = false;

    public void init(HardwareMap hwMap)
    {
        //Initializes motors
        leftFrontWheel = hwMap.get(DcMotor.class, "flWheel");
        rightFrontWheel = hwMap.get(DcMotor.class, "frWheel");
        leftBackWheel = hwMap.get(DcMotor.class, "blWheel");
        rightBackWheel = hwMap.get(DcMotor.class, "brWheel");

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parametres = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parametres);
        imu.resetYaw();
        //Default run mode
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Reverse right motors because they are mirrored
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorSimple.Direction.FORWARD);
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
                StartYawLock();
                yawLockStarted = true;
            }
            if (yawLocked) {PowerWheels(fedAxial, fedLateral, P+D, denominator);}
            else {PowerWheels(fedAxial, fedLateral, yaw, denominator);}
            UpdateYawLock();
        }
        else
        {
            yawLocked = false;
            yawLockStarted = false;
            PowerWheels(fedAxial, fedLateral, yaw, denominator);
        }
        oldError = error;
        PowerWheels(axial, lateral, yaw, denominator);
    }

    public void PowerWheels(double axial, double lateral, double yaw, double denominator)
    {
        leftFrontWheel.setPower(((axial - lateral - yaw) / denominator));
        leftBackWheel.setPower(((axial + lateral - yaw) / denominator));
        rightFrontWheel.setPower(((axial + lateral + yaw) / denominator));
        rightBackWheel.setPower(((axial - lateral + yaw) / denominator));
    }

    public void SwitchDriveMode() {fieldCentric = !fieldCentric; IMUAngle = GetHeading();}

    public double GetHeading()
    {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    public void ResetIMU() {imu.resetYaw(); IMUAngle = 0;}
    

    enum YawLockState {
        IDLE,
        LOCK_YAW
    }

    public void StartYawLock()
    {
        yawLockState = YawLockState.LOCK_YAW;
        yawLockTimer.reset();
    }

    public void UpdateYawLock()
    {
        switch (yawLockState) {
            case LOCK_YAW:
                if (yawLockTimer.milliseconds() > 200) {
                    IMUAngle = GetHeading();
                    yawLocked = true;
                    yawLockTimer.reset();
                    yawLockState = YawLockState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }
}