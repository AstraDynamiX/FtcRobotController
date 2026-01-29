package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.CameraBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp_DECODE")
public class TeleOp extends OpMode {

    private final double MOTOR_MULTIPLIER = 0.85;
    private final double STRAFE_MULTIPLIER = 1.4;

    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();
    CameraBoard CamBoard = new CameraBoard();

    //Flags for buttons
    private boolean buttonHeld = false;
    ElapsedTime buttonHoldTimer = new ElapsedTime();

    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean rightHeld = false;
    private boolean leftHeld = false;
    private boolean options1Held = false;
    private boolean xHeld = false;
    private boolean yHeld = false;

    private int launchState = 0;
    private double flywheelMultiplier = 0.65;
    private boolean camAdjustment = false;

    @Override
    public void init()
    {
        OmniBoard.init(hardwareMap);
        LaunchBoard.init(hardwareMap);
        telemetry.addData("BOOTED:", "Welcome to AstraDynamiX Technologies!");
    }

    @Override
    public void loop()
    {
        OmniBoard.ChassisMovement(
                gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.left_stick_x * (MOTOR_MULTIPLIER*STRAFE_MULTIPLIER) / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
        );

        /*if (gamepad1.options && !options1Held)
        {
            OmniBoard.SwitchDriveMode();
            options1Held = true; fieldCentric = !fieldCentric;
        }
        if (!gamepad1.options) {options1Held = false;}

        if (gamepad1.share) {OmniBoard.ResetIMU();}*/

        //Toggle cam adjustment
        if (gamepad1.options && !options1Held)
        {
            options1Held = true;
            camAdjustment = !camAdjustment;
        }
        if (!gamepad1.options) {options1Held = false;}

        //Outtake
        if (gamepad1.a && !aHeld)
        {
            yHeld = true;
            LaunchBoard.IntakeMovement(-0.8);
        }
        if(!gamepad1.y) {yHeld = false;}

        //Shooting
        if (gamepad1.left_bumper && !leftBumperHeld)
        {
            leftBumperHeld = true;
            switch (launchState)
            {
                case 0: LaunchBoard.Intake(); break;
                case 1: LaunchBoard.Rev(); break;
            }
            launchState++;
            if (launchState == 2) {launchState = 0;}
        }
        if(!gamepad1.left_bumper) {leftBumperHeld = false;}

        if (gamepad1.right_bumper && !rightBumperHeld)
        {
            rightBumperHeld = true;
            LaunchBoard.Shoot();
        }
        if(!gamepad1.right_bumper) {rightBumperHeld = false;}

        if (gamepad1.y) {LaunchBoard.Idle();}

        //Flywheel power adjustment
        if (gamepad1.dpad_up && !upHeld)
        {
            upHeld = true;
            flywheelMultiplier += 0.05;
        }
        if (!gamepad1.dpad_up) {upHeld = false;}

        if (gamepad1.dpad_down && !downHeld)
        {
            downHeld = true;
            flywheelMultiplier -= 0.05;
        }
        if (!gamepad1.dpad_down) {downHeld = false;}

        if (gamepad1.dpad_right && !rightHeld)
        {
            rightHeld = true;
            LaunchBoard.AngleAdjusterMovement(-0.05);
        }
        if (!gamepad1.dpad_right) {rightHeld = false;}

        if (gamepad1.dpad_left && !leftHeld)
        {
            leftHeld = true;
            LaunchBoard.AngleAdjusterMovement(0.05);
        }
        if (!gamepad1.dpad_left) {leftHeld = false;}

        //Functions that get called every loop with no conditions
        LaunchBoard.UpdateLaunch(camAdjustment, flywheelMultiplier);
        LaunchBoard.UpdateAngleAdjuster();
        if (camAdjustment) {LaunchBoard.TurretMovement();}
        else {LaunchBoard.TurretLockPosition(0);}

        // ------ Telemetry ------
        telemetry.addData("CAM ADJUSTMENT", camAdjustment);
        if (camAdjustment)
        {
            telemetry.addData("APRIL TAG DISTANCE", LaunchBoard.getAprilTagDistance());
            telemetry.addData("APRIL TAG BEARING", LaunchBoard.getAprilTagBearing());
            telemetry.addData("LAUNCH ANGLE", LaunchBoard.getLaunchAngle());
            telemetry.addData("", "");
            telemetry.addData("TURRET POSITION", LaunchBoard.getTurretPosition());
        }
        else
        {
            telemetry.addData("FLYWHEEL MULTIPLIER", flywheelMultiplier);
            telemetry.addData("LAUNCH ANGLE", LaunchBoard.getAdjusterAngle());
        }
        //telemetry.addData("IMU:", OmniBoard.GetHeading() / 3.1415 + "π");
    }

    // ------ state machines ------

    public boolean UpdateButtonHold(boolean button)
    {
        if (buttonHeld)
        {
            if (buttonHoldTimer.milliseconds() >= 350 && button) {return true;}
            else
            {
                if (!button) {buttonHeld = false;}
                return false;
            }
        }
        else
        {
            if (button)
            {
                buttonHoldTimer.reset();
                buttonHeld = true;
            }
            return false;
        }
    }

}

