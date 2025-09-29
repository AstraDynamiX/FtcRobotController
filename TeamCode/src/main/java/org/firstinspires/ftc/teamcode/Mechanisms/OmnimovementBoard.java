package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class OmnimovementBoard
{
    // Left front, left back, right front, right back
    private final double[] GEAR_RATIOS = {1, 1, 1, 1};
    private final double IMU_KP = 2.5;
    private final double IMU_KD = 5;

    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftBackWheel;
    private DcMotor rightBackWheel;
    private IMU imu;

    private LinearOpMode opMode;
    private ElapsedTime timer = new ElapsedTime();

    private double previousHeading = 0;
    private double integratedHeading = 0;
    private double IMUAngle = 0;

    private boolean fieldCentric = false;


    public void init(HardwareMap hwMap)
    {
        //Initializes motors
        leftFrontWheel = hwMap.get(DcMotor.class, "flWheel");
        rightFrontWheel = hwMap.get(DcMotor.class, "frWheel");
        leftBackWheel = hwMap.get(DcMotor.class, "blWheel");
        rightBackWheel = hwMap.get(DcMotor.class, "brWheel");

        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parametres = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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

    public void initAuto(LinearOpMode opMode) {this.opMode = opMode;}

    public void ChassisMovement(double axial, double lateral, double yaw, double maxSpeed)
    {
        //Keeps ratio if power exceeds motor multiplier
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), maxSpeed) / maxSpeed;

        if (fieldCentric)
        {
            double heading = getIntegratedHeading() - 90;
            double adjustedAxial = lateral * Math.cos(heading) - axial * Math.sin(heading);
            double adjustedLateral = lateral * Math.sin(heading) + axial * Math.cos(heading);
            PowerWheels(adjustedAxial, adjustedLateral, yaw, denominator);
        }
        else
        {
            PowerWheels(axial, lateral, yaw, denominator);
        }
    }

    public void SwitchDriveMode() {fieldCentric = !fieldCentric;}


    public void IMUFollow(double duration, double axial, double lateral)
    {
        double oldError = 0;
        timer.reset();
        while (timer.milliseconds() < duration && !opMode.isStopRequested())
        {
            double error = getIntegratedHeading() - IMUAngle;
            double P = IMU_KP * error;
            double D = IMU_KD * (error - oldError);
            PowerWheels(axial, lateral, P+D, 100);
            oldError = error;
        }
        PowerWheels(0, 0, 0, 0);
    }

    public void ChangeIMUAngle(double angle)
    {
        IMUAngle = angle; imu.resetYaw();
        if (IMUAngle < 0) {IMUFollow(-IMUAngle * 13.5,0, 0);}
        else {IMUFollow(IMUAngle * 13.5,0, 0);}
    }

    // ------ General functions ------

    public void PowerWheels(double axial, double lateral, double yaw, double denominator)
    {
        leftFrontWheel.setPower(((axial + lateral + yaw) / denominator) * GEAR_RATIOS[0]);
        leftBackWheel.setPower(((axial - lateral + yaw) / denominator) * GEAR_RATIOS[1]);
        rightFrontWheel.setPower(((axial - lateral - yaw) / denominator) * GEAR_RATIOS[2]);
        rightBackWheel.setPower(((axial + lateral - yaw) / denominator) * GEAR_RATIOS[3]);
    }

    public double getIntegratedHeading()
    {
        //Delta heading exists because loop gets called only every 50 milliseconds
        double currentHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        //Normalizes degrees to range from -180 to 180
        if (integratedHeading < -180) {
            integratedHeading += 360;
        } else if (integratedHeading >= 180) {
            integratedHeading -= 360;
        }

        return integratedHeading;
    }

}