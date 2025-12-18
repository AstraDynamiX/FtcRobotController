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
    //RevState revState = RevState.IDLE;
    //ButtonHoldState buttonHoldState = ButtonHoldState.IDLE;
    ElapsedTime revTimer = new ElapsedTime();
    ElapsedTime buttonHoldTimer = new ElapsedTime();

    //Flags for buttons
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean aHeld = false;
    private boolean bHeld = false;
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

        Global.pattern = new double[]{2, 1, 1};
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.x) {Global.pattern = new double[]{2, 1, 1};}
        if (gamepad1.a) {Global.pattern = new double[]{1, 2, 1};}
        if (gamepad1.b) {Global.pattern = new double[]{1, 1, 2};}

        double pattern = Global.pattern[0] * 100 + Global.pattern[1] * 10 + Global.pattern[2];
        telemetry.addData("PATTERN:", pattern);
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

        /*if (gamepad1.options && !options1Held)
        {
            OmniBoard.SwitchDriveMode();
            options1Held = true; fieldCentric = !fieldCentric;
        }
        if (!gamepad1.options) {options1Held = false;}

        if (gamepad1.share) {OmniBoard.ResetIMU();}*/

        // ------ Mechanism controls ------

        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            LaunchBoard.SwitchMode();
        }
        if (!gamepad1.a) {aHeld = false;}

        //Flywheel revving and low power mode
        /*if (gamepad1.left_bumper && !leftBumperHeld)
        {
            if (revState == RevState.IDLE) {revState = RevState.REVVING;}
            else {lowPowerFlywheel = !lowPowerFlywheel;}
            leftBumperHeld = true;
        }
        if (!gamepad1.left_bumper) {leftBumperHeld = false;}
        if (UpdateButtonHold(gamepad1.left_bumper)) {revState = RevState.IDLE;}*/

        //Comments
        telemetry.addData("FIELD CENTRIC", fieldCentric);
        double indexerSlots = Global.indexerSlots[0] * 100 + Global.indexerSlots[1] * 10 + Global.indexerSlots[2];
        telemetry.addData("SLOT VALUES", indexerSlots);
        telemetry.addData("CURRENT SLOT", Global.currentSlot);
        telemetry.addData("IS SHOOTING", LaunchBoard.isShooting);
        telemetry.addData("INDEXER SPINNING", LaunchBoard.GetIndexerState());
        telemetry.addData("MUST REACH ZERO", LaunchBoard.GetMustReachZero());
        telemetry.addData("", "");
        telemetry.addData("CURRENT INDEXER ANGLE", LaunchBoard.GetIndexerAngle());
        telemetry.addData("TARGET INDEXER ANGLE", LaunchBoard.GetTargetIndexerAngle());
        telemetry.addData("INDEXER POWER", LaunchBoard.GetIndexerPower());
        //telemetry.addData("IMU:", OmniBoard.GetHeading() / 3.141 + "π");
        //telemetry.addData("CURRENT TICK ROT:", LaunchBoard.GetCurrentTickRotations());
        //telemetry.addData("TARGET TICK ROT:", LaunchBoard.GetCurrentTickRotations());

        //UpdateRev();
        LaunchBoard.UpdateIndexerSpin();
        LaunchBoard.UpdateIntake();
        LaunchBoard.UpdateShooting();
    }

    // ------ state machines ------

    /*enum ButtonHoldState
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
                else
                {
                    if (!buttonHeld) {buttonHoldState = ButtonHoldState.IDLE;}
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
    }

    public void UpdateRev()
    {
        switch (revState)
        {
            case REVVING:

                if (revTimer.milliseconds() >= 800) {isFlywheelReady = true;}
                LaunchBoard.FlywheelMovement(0.935 * ((lowPowerFlywheel) ? 0.65 : 1));
                break;

            case IDLE:
            default:

                LaunchBoard.FlywheelMovement(0);
                isFlywheelReady = false;
                revTimer.reset();
                break;
        }
    }*/

}

