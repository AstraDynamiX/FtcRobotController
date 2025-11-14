package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp_DECODE")
public class TeleOp extends OpMode {

    private final double MOTOR_MULTIPLIER = 0.8;
    private final double STRAFE_MULTIPLIER = 1.4;

    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();
    //State machines
    LaunchState launchState = LaunchState.IDLE;
    RevState revState = RevState.IDLE;
    ButtonHoldState buttonHoldState = ButtonHoldState.IDLE;
    ElapsedTime launchTimer = new ElapsedTime();
    ElapsedTime revTimer = new ElapsedTime();
    ElapsedTime buttonHoldTimer = new ElapsedTime();

    //Flags for buttons
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean aHeld = false;
    private boolean options1Held = false;
    //Flags for mechanisms
    private boolean lowPowerFlywheel = true;
    private boolean isFlywheelReady = false;
    private boolean isIntakeOn = false;
    //Other flags
    private boolean fieldCentric = false;


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
        // ------ Chassis movement ------

        OmniBoard.ChassisMovement(
                gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.left_stick_x * (MOTOR_MULTIPLIER*STRAFE_MULTIPLIER) / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger+1) * (gamepad1.right_trigger/2+1),
                MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
        );

        if (gamepad1.options && !options1Held)
        {
            OmniBoard.SwitchDriveMode();
            options1Held = true; fieldCentric = !fieldCentric;
        }
        if (!gamepad1.options) {options1Held = false;}

        if (gamepad1.share) {OmniBoard.ResetIMU();}

        // ------ Mechanism controls ------

        //Intake
        if (gamepad1.a && !aHeld)
        {
            if (!isIntakeOn) {LaunchBoard.IntakeMovement(-0.65);}
            else {LaunchBoard.IntakeMovement(0);}
            isIntakeOn = !isIntakeOn;
            aHeld = true;
        }
        if (!gamepad1.a) {aHeld = false;}

        //Flywheel revving and low power mode
        if (gamepad1.left_bumper && !leftBumperHeld)
        {
            if (revState == RevState.IDLE) {StartRev();}
            else
            {
                lowPowerFlywheel = !lowPowerFlywheel;
                leftBumperHeld = true;
            }
        }
        if (!gamepad1.left_bumper) {leftBumperHeld = false;}
        if (UpdateButtonHold(gamepad1.left_bumper)) StopRev();

        //Launching the balls
        if (gamepad1.right_bumper && !rightBumperHeld)
        {
            if (launchState == LaunchState.IDLE) {StartLaunch();}
            rightBumperHeld = true;
        }
        if (!gamepad1.right_bumper) {rightBumperHeld = false;}

        //Comments
        telemetry.addData("FLYWHEEL:", (isFlywheelReady) ? ((lowPowerFlywheel) ? "low power" : "ready") : "not ready");
        telemetry.addData("FIELD CENTRIC:", fieldCentric);
        telemetry.addData("IMU:", OmniBoard.getIntegratedHeading());

        UpdateRev();
        UpdateLaunch();
    }

    // ------ Launching and revving state machines ------

    enum ButtonHoldState
    {
        IDLE,
        PRESSED
    }

    public boolean UpdateButtonHold(boolean buttonHeld)
    {
        switch (buttonHoldState)
        {

            case PRESSED :

                if (buttonHoldTimer.milliseconds() >= 350 && buttonHeld) {return true;}
                else if (!buttonHeld)
                {
                    buttonHoldState = ButtonHoldState.IDLE;
                    return false;
                }

            case IDLE :
            default :

                if (buttonHeld)
                {
                    buttonHoldTimer.reset();
                    buttonHoldState = ButtonHoldState.PRESSED;
                }
                return false;
        }
    }

    enum RevState
    {
        IDLE,
        REVVING,
        REVVED
    }

    public void StartRev() {revState = RevState.REVVING;}
    public void StopRev() {revState = RevState.IDLE;}

    public void UpdateRev()
    {
        switch (revState)
        {
            case REVVING:

                LaunchBoard.FlywheelMovement(0.9 * ((lowPowerFlywheel) ? 0.6 : 1));
                revTimer.reset();
                revState = RevState.REVVED;
                break;

            case REVVED:

                if (revTimer.milliseconds() >= 800) {isFlywheelReady = true;}
                LaunchBoard.FlywheelMovement(0.9 * ((lowPowerFlywheel) ? 0.6 : 1));
                break;

            case IDLE:
            default:

                LaunchBoard.FlywheelMovement(0);
                isFlywheelReady = false;
                break;
        }
    }


    enum LaunchState
    {
        IDLE,
        LAUNCHING,
        LAUNCHED,
    }

    public void StartLaunch() {launchState = LaunchState.LAUNCHING;}

    public void UpdateLaunch()
    {
        switch (launchState)
        {
            case LAUNCHING:

                LaunchBoard.BallStopMovement(0.425);
                launchTimer.reset();
                launchState = LaunchState.LAUNCHED;

            case LAUNCHED:

                if (launchTimer.milliseconds() >= 450)
                {
                    if (!gamepad1.right_bumper)
                    {
                        LaunchBoard.BallStopMovement(0.3125);
                        launchState = LaunchState.IDLE;
                    }
                }
                break;

            case IDLE:
            default:

                break;
        }
    }

}

