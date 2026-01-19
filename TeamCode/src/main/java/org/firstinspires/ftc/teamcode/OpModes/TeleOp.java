package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mechanisms.CameraBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;
import java.util.List;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp_DECODE")
public class TeleOp extends OpMode {

    private final double MOTOR_MULTIPLIER = 0.85 * 11 / 8;
    private final double STRAFE_MULTIPLIER = 1.4;

    public static double FLYWHEEL_CAMERA_WEIGHT = 0.0085;
    public static double FLYWHEEL_CAMERA_BIAS = 30;

    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();
    CameraBoard CamBoard = new CameraBoard();

    FlywheelState flywheelState = FlywheelState.IDLE;
    private boolean isShooting = false;
    private boolean intakeOn = false;

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
    //Other flags
    private boolean fieldCentric = false;

    private double batteryVoltage = 12.7;
    private double flywheelMultiplier = 0.8;
    private double power;
    private boolean camAdjustment = false;
    // Auto strafe variables
    private AutoStrafeState autoStrafeState = AutoStrafeState.IDLE;
    private double lockedHeading = 0;
    private ElapsedTime autoStrafeTimer = new ElapsedTime();
    public static double AUTO_STRAFE_TIME = 1.5; // secunde
    public static double AUTO_STRAFE_POWER = 0.6; // putere strafe

    // Flag pentru gamepad2.y
    private boolean yHeld2 = false;
    @Override
    public void init()
    {
        OmniBoard.init(hardwareMap);
        LaunchBoard.init(hardwareMap);
        CamBoard.init(hardwareMap);
        telemetry.addData("BOOTED:", "Welcome to AstraDynamiX Technologies!");

        Global.pattern = new double[]{2, 1, 1};
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.x) {Global.pattern = new double[]{2, 1, 1};}
        if (gamepad1.a) {Global.pattern = new double[]{1, 2, 1};}
        if (gamepad1.b) {Global.pattern = new double[]{1, 1, 2};}

        if (gamepad1.dpad_up && !upHeld)
        {
            upHeld = true;
            batteryVoltage += 0.1;
        }
        if (!gamepad1.dpad_up) {upHeld = false;}

        if (gamepad1.dpad_down && !downHeld)
        {
            downHeld = true;
            batteryVoltage -= 0.1;
        }
        if (!gamepad1.dpad_down) {downHeld = false;}

        double pattern = Global.pattern[0] * 100 + Global.pattern[1] * 10 + Global.pattern[2];
        telemetry.addData("PATTERN", pattern);
        telemetry.addData("BATTERY VOLTAGE", batteryVoltage);
    }

    @Override
    public void loop()
    {
        if (autoStrafeState == AutoStrafeState.IDLE) {
            OmniBoard.ChassisMovement(
                    gamepad1.left_stick_y * MOTOR_MULTIPLIER / (gamepad1.left_trigger + 1) * (gamepad1.right_trigger / 2 + 1),
                    gamepad1.left_stick_x * (MOTOR_MULTIPLIER * STRAFE_MULTIPLIER) / (gamepad1.left_trigger + 1) * (gamepad1.right_trigger / 2 + 1),
                    gamepad1.right_stick_x * MOTOR_MULTIPLIER / (gamepad1.left_trigger + 1) * (gamepad1.right_trigger / 2 + 1),
                    MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
            );
        }
        if (gamepad2.y && !yHeld2)
        {
            yHeld = true;
            StartAutoStrafe();
        }
        if (!gamepad2.y) {yHeld2 = false;}

// Update auto strafe state machine
        UpdateAutoStrafe();

        /*if (gamepad1.options && !options1Held)
        {
            OmniBoard.SwitchDriveMode();
            options1Held = true; fieldCentric = !fieldCentric;
        }
        if (!gamepad1.options) {options1Held = false;}

        if (gamepad1.share) {OmniBoard.ResetIMU();}*/

        // ------ Mechanism controls ------

        if (gamepad1.options && !options1Held)
        {
            camAdjustment = !camAdjustment;
            options1Held = true;
        }
        if (!gamepad1.options) {options1Held = false;}


        //Flywheel
        if (gamepad1.right_bumper && !rightBumperHeld)
        {
            rightBumperHeld = true;
            if (flywheelState == FlywheelState.IDLE) {flywheelState = FlywheelState.FULL_POWER;}
            else if (flywheelState == FlywheelState.FULL_POWER) {flywheelState = FlywheelState.LOW_POWER;}
            else if (flywheelState == FlywheelState.LOW_POWER) {flywheelState = FlywheelState.FULL_POWER;}
        }
        if (!gamepad1.right_bumper) {rightBumperHeld = false;}
        if (UpdateButtonHold(gamepad1.right_bumper)) {flywheelState = FlywheelState.IDLE;}

        //Intake
        if (gamepad1.left_bumper && !leftBumperHeld)
        {
            leftBumperHeld = true;
            if (intakeOn) {LaunchBoard.IntakeMovement(0);}
            else {LaunchBoard.IntakeMovement(-0.9);}
            intakeOn = !intakeOn;
        }
        if (!gamepad1.left_bumper) {leftBumperHeld = false;}

        if (gamepad1.b && !bHeld)
        {
            bHeld = true;
            if (intakeOn) {LaunchBoard.IntakeMovement(0);}
            else {LaunchBoard.IntakeMovement(0.75);}
            intakeOn = !intakeOn;
        }
        if (!gamepad1.b) {bHeld = false;}

        if (gamepad1.x && !xHeld)
        {
            xHeld = true;
            LaunchBoard.LittleOuttake(0.8);
        }
        if(!gamepad1.x) {xHeld = false;}
        LaunchBoard.updateLittleOuttake();

        //Shooting
        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            LaunchBoard.StartShooting();
        }
        if (!gamepad1.a) {aHeld = false;}

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
            flywheelMultiplier += 0.01;
        }
        if (!gamepad1.dpad_right) {rightHeld = false;}

        if (gamepad1.dpad_left && !leftHeld)
        {
            leftHeld = true;
            flywheelMultiplier -= 0.01;
        }
        if (!gamepad1.dpad_left) {leftHeld = false;}


        //Comments
        //telemetry.addData("FIELD CENTRIC", fieldCentric);
        //double indexerSlots = Global.indexerSlots[0] * 100 + Global.indexerSlots[1] * 10 + Global.indexerSlots[2];
        //telemetry.addData("SLOT VALUES", indexerSlots);
        //telemetry.addData("CURRENT SLOT", Global.currentSlot);
        telemetry.addData("FLYWHEEL STATE", (flywheelState == FlywheelState.FULL_POWER) ? "full power" : "low power");
        telemetry.addData("FLYWHEEL MULTIPLIER", flywheelMultiplier);
        telemetry.addData("", "");
        telemetry.addData("FLYWHEEL POWER", power*0.875);
        telemetry.addData("CAM ADJUSTMENT", camAdjustment);
        if (!CamBoard.GetAprilTag().isEmpty())
        {telemetry.addData("APRIL TAG DISTANCE", CamBoard.GetAprilTag().get(0));}
        //telemetry.addData("IMU:", OmniBoard.GetHeading() / 3.141 + "π");
        //telemetry.addData("CURRENT TICK ROT:", LaunchBoard.GetCurrentTickRotations());
        //telemetry.addData("TARGET TICK ROT:", LaunchBoard.GetCurrentTickRotations());

        UpdateFlywheel();
        LaunchBoard.UpdateIntake();
        LaunchBoard.UpdateShooting();
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

    enum FlywheelState
    {
        IDLE,
        FULL_POWER,
        LOW_POWER
    }
    enum AutoStrafeState
    {
        IDLE,
        LOCK_HEADING,
        STRAFE_LEFT,
        COMPLETE
    }

    public void UpdateFlywheel()
    {
        if (camAdjustment)
        {
            List<Double> tags = CamBoard.GetAprilTag();
            if (!tags.isEmpty()) {power =
                    -0.01 * FLYWHEEL_CAMERA_BIAS +
                    -FLYWHEEL_CAMERA_WEIGHT * flywheelMultiplier *
                    14 / batteryVoltage *
                    tags.get(0);
            }
        }
        else
        {power = -flywheelMultiplier * 14 / batteryVoltage;}

        switch (flywheelState)
        {
            case FULL_POWER:

                LaunchBoard.FlywheelMovement(power*0.9);
                break;

            case LOW_POWER:

                LaunchBoard.FlywheelMovement(power*0.4);
                break;

            case IDLE:
            default:

                LaunchBoard.FlywheelMovement(0);
                break;
        }
    }
    private void StartAutoStrafe()
    {
        // aici face cacat de strafe
        lockedHeading = OmniBoard.GetHeading();

        autoStrafeState = AutoStrafeState.STRAFE_LEFT;
        autoStrafeTimer.reset();

        telemetry.addData("AUTO STRAFE", "Started! Heading locked at %.1f°", Math.toDegrees(lockedHeading));
    }

    /**
     * Update pentru auto strafe state machine
     */
    private void UpdateAutoStrafe()
    {
        switch (autoStrafeState)
        {

            case STRAFE_LEFT:
                // Calculează eroarea de heading
                double currentHeading = OmniBoard.GetHeading();
                autoStrafeTimer.startTime();

                // Aplică mișcarea: strafe la stânga cu heading lock
                OmniBoard.ChassisMovement(
                        0,                      // nu merge înainte/înapoi
                        -AUTO_STRAFE_POWER,     // strafe la stânga
                        0,      // corectează heading-ul
                        MOTOR_MULTIPLIER * STRAFE_MULTIPLIER
                );

                // Verifică dacă a trecut timpul
                if (autoStrafeTimer.seconds() >= AUTO_STRAFE_TIME)
                {
                    autoStrafeState = AutoStrafeState.COMPLETE;
                    autoStrafeTimer.reset();
                }
                break;

            case COMPLETE:
                // Oprește motoarele
                OmniBoard.ChassisMovement(0, 0, 0, MOTOR_MULTIPLIER);

                // Așteaptă puțin înainte de clear
                if (autoStrafeTimer.milliseconds() > 200)
                {
                    ClearAutoStrafe();
                }
                break;

            case IDLE:
            default:
                // Control manual normal - nu face nimic
                break;
        }
    }
    private void ClearAutoStrafe()
    {
        autoStrafeState = AutoStrafeState.IDLE;
        lockedHeading = 0;

        telemetry.addData("AUTO STRAFE", "Complete! Manual control restored.");
    }

    /**
     * Helper pentru angle wrapping
     */
    private double AngleWrap(double angle)
    {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}

