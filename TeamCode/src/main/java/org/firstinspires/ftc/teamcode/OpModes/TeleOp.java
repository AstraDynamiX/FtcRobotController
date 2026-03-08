package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp_DECODE")
public class TeleOp extends OpMode {

    private final double MOTOR_MULTIPLIER = 0.875;
    private final double STRAFE_MULTIPLIER = 1.4;

    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    //Flags for buttons
    private boolean buttonHeld = false;
    ElapsedTime buttonHoldTimer = new ElapsedTime();

    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean rightHeld = false;
    private boolean leftHeld = false;
    //Init loop booleans
    private boolean redAlliance = false;
    private boolean allianceConfirmed = false;

    private int launchState = 0;
    private double flywheelMultiplier = 300;
    private boolean camAdjustment = true;
    private boolean robotCentricDrive = true;
    private boolean outtake = false;


    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        
        telemetry.addData("BOOTED:", "Welcome to AstraDynamiX Technologies!");
    }

    @Override
    public void init_loop()
    {
        if (allianceConfirmed)
        {telemetry.addData("CONFIRMED", (redAlliance) ? "red" : "blue");}
        else
        {
            telemetry.addData("", "() - change alliance, X - confirm");

            if (gamepad1.bWasPressed())
            {redAlliance = !redAlliance;}

            telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");

            if (gamepad1.a)
            {
                allianceConfirmed = true;
                LaunchBoard.init(hardwareMap, redAlliance);
            }
        }
    }

    @Override
    public void start()
    {
        LaunchBoard.start();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop()
    {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1),
                -gamepad1.left_stick_x * MOTOR_MULTIPLIER * STRAFE_MULTIPLIER / (gamepad1.left_trigger+1),
                -gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1),
                robotCentricDrive
        );
        follower.update();
        /*OmniBoard.ChassisMovement(
                gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.left_stick_x * (MOTOR_MULTIPLIER*STRAFE_MULTIPLIER) / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
        );*/

        if (gamepad1.optionsWasPressed())
        {camAdjustment = !camAdjustment;}

        if (gamepad1.shareWasPressed())
        {robotCentricDrive = !robotCentricDrive;}

        //Shooting
        if (gamepad1.leftBumperWasPressed())
        {
            switch (launchState)
            {
                case 0: LaunchBoard.Intake(); break;
                case 1: LaunchBoard.Rev(); break;
            }
            launchState++;
            if (launchState == 2) {launchState = 0;}
        }

        if(gamepad1.x)
        {
            outtake = true;
            LaunchBoard.Outtake();
        }
        if (!gamepad1.x && outtake)
        {
            outtake = false;
            LaunchBoard.Intake();
        }

        if (gamepad1.rightBumperWasPressed())
        {LaunchBoard.Shoot();}

        if (gamepad1.y)
        {LaunchBoard.Idle();}

        //Flywheel and angle adjustment
        if (gamepad1.dpad_up && !upHeld)
        {
            upHeld = true;
            flywheelMultiplier += 25;
        }
        if (!gamepad1.dpad_up) {upHeld = false;}

        if (gamepad1.dpad_down && !downHeld)
        {
            downHeld = true;
            flywheelMultiplier -= 25;
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
        if (camAdjustment)
        {LaunchBoard.TurretMovement();}
        else
        {LaunchBoard.TurretLockPosition(0);}

        // ------ Telemetry ------
        telemetry.addData("ROBOT CENTRIC", robotCentricDrive);
        telemetry.addData("CAM ADJUSTMENT", camAdjustment);
        telemetry.addData("LAUNCH ANGLE", LaunchBoard.getLaunchAngle());
        telemetry.addData("LAUNCH SPEED", LaunchBoard.getFlywheelSpeed());

        if (!camAdjustment)
        {telemetry.addData("FLYWHEEL MULTIPLIER", flywheelMultiplier);}
    }

    @Override
    public void stop()
    {
        LaunchBoard.stop();
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

