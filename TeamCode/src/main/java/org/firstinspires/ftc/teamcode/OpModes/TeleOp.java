package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "NORMAL_TeleOp_DECODE")
public class TeleOp extends OpMode {

    private final double MOTOR_MULTIPLIER = 0.8;
    private final double STRAFE_MULTIPLIER = 1.4;

    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();
    //State machines (advanced programming type shi)
    LaunchState launchState = LaunchState.IDLE;
    RevState revState = RevState.IDLE;
    ElapsedTime launchTimer = new ElapsedTime();
    ElapsedTime revTimer = new ElapsedTime();

    //Flags for buttons
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean options1Held = false;
    //Flags for mechanisms
    private boolean isMidGeckoReady = false;
    private int ballsLaunched = 0;
    //Other flags
    private boolean fieldCentric = false;


    @Override
    public void init()
    {
        OmniBoard.init(hardwareMap); LaunchBoard.init(hardwareMap);
        telemetry.addData("BOOTED:", "Welcome to AstraDynamiX Technologies!");
    }

    @Override
    public void loop()
    {
        // ------ Chassis movement ------

        OmniBoard.ChassisMovement(
                -gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger/2.5+1),
                gamepad1.left_stick_x * (MOTOR_MULTIPLIER*STRAFE_MULTIPLIER) / (gamepad1.left_trigger/2.5+1),
                gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger/2.5+1),
                MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
        );
        if (gamepad1.options && !options1Held)
        {
            OmniBoard.SwitchDriveMode();
            options1Held = true; fieldCentric = !fieldCentric;
        }
        if (!gamepad1.options) {options1Held = false;}

        // ------ Mechanism controls ------

        //Separate midGecko revving
        if (gamepad1.left_bumper && !leftBumperHeld)
        {
            if (revState == RevState.IDLE) {StartRev();}
            else {StopRev();}
            leftBumperHeld = true;
        }
        if (!gamepad1.left_bumper) {leftBumperHeld = false;}

        //Launching the balls
        if (gamepad1.right_bumper && !rightBumperHeld)
        {
            if (launchState == LaunchState.IDLE) {StartLaunch();}
            rightBumperHeld = true;
        }
        if (!gamepad1.right_bumper) {rightBumperHeld = false;}

        if (gamepad1.b)
        {LaunchBoard.SideGeckoMovement(0);}

        //Comments
        telemetry.addData("MID GECKO READY:", isMidGeckoReady);
        telemetry.addData("FIELD CENTRIC:", fieldCentric);

        UpdateRev();
        UpdateLaunch();
    }

    // ------ Launching and revving state machines ------

    //This exists so that instead of blocking the entire code with a while loop that
    //checks the timer, there's a function that checks the timer with multiple if
    //statements. Every time an if statement is triggered, the case switches.

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

                LaunchBoard.MidGeckoMovement(0.85);
                revTimer.reset();
                revState = RevState.REVVED;
                break;

            case REVVED:

                if (revTimer.milliseconds() >= 800) {isMidGeckoReady = true;}
                break;

            case IDLE:
            default:

                LaunchBoard.MidGeckoMovement(0);
                isMidGeckoReady = false;
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

                LaunchBoard.SideGeckoMovement(0.85);
                launchTimer.reset();
                launchState = LaunchState.LAUNCHED;

            case LAUNCHED:

                if (launchTimer.milliseconds() >= 650)
                {
                    LaunchBoard.SideGeckoMovement(0);
                    ballsLaunched++;
                    if (ballsLaunched == 3) {ballsLaunched = 0;}
                    launchState = LaunchState.IDLE;
                }
                break;

            case IDLE:
            default:

                break;
        }
    }

}

