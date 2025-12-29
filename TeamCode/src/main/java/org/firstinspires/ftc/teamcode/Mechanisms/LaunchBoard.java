package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModes.Global;

public class LaunchBoard
{
    private final double FLYWHEEL_KP = 0;
    private final double FLYWHEEL_KI = 0;
    private final double FLYWHEEL_KD = 0;

    private final double INDEXER_KP = 4.5;
    private final double INDEXER_KI = 0.025;
    private final double INDEXER_KD = 2.675;

    private double INDEXER_TURN = 1;

    private DcMotor intake;
    private DcMotor flywheel;
    private Servo transfer;
    private CRServo indexer;

    public ColorSensor ballColor;
    public DistanceSensor ballDistance;
    private AnalogInput indexerAngle;

    public boolean isShooting = true; //Start with intake

    IntakeState intakeState = IntakeState.IDLE;
    ElapsedTime intakeTimer = new ElapsedTime();
    private boolean ballEntered = false;

    ShootingState shootingState = ShootingState.IDLE;
    ElapsedTime transferTimer = new ElapsedTime();

    private boolean indexerSpinning = false;
    private double targetIndexerAngle = INDEXER_TURN * 0.3; //Offset
    private boolean mustReachZero = true;

    ElapsedTime flywheelTime = new ElapsedTime();
    private double prevFlywheelSnapshot = 0;
    private double oldFlywheelError = 0;
    private double currentSnapshotRps;
    private double lastSnapshotRps = 0;
    private double flywheelI = 0;

    private double oldIndexerError = 0;
    private double indexerP = 0;
    private double indexerI = 0;
    private double indexerD = 0;

    private double indexerInput = 0;
    public double adjustedAngle;


    public void init(HardwareMap hwMap)
    {
        Global.indexerSlots = new double[]{0, 0, 0};
        Global.currentPatternSlot = 0;
        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        transfer = hwMap.get(Servo.class, "transfer");
        indexer = hwMap.get(CRServo.class, "indexer");

        ballColor = hwMap.get(ColorSensor.class, "ballColor");
        ballDistance = hwMap.get(DistanceSensor.class, "ballDistance");
        indexerAngle = hwMap.get(AnalogInput.class, "indexerAngle");
        //Default run mode
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetIndexerAngle += INDEXER_TURN / 2;
        SwitchMode();
        intakeTimer.startTime();
    }

    public void SwitchMode()
    {
        isShooting = !isShooting;
        if (isShooting)
        {
            targetIndexerAngle += (INDEXER_TURN / 2);
            intakeState = IntakeState.IDLE;
            shootingState = ShootingState.TRANSFER_UP;
        }
        else
        {
            FlywheelMovement(0);
            targetIndexerAngle -= (INDEXER_TURN / 2);
            if (intakeState != IntakeState.FULL) {intakeState = IntakeState.EATING;}
            shootingState = ShootingState.IDLE;
        }
        targetIndexerAngle -= INDEXER_TURN; //Gets added back when calling StartIndexerSpin();
        StartIndexerSpin();
    }

    public void FlywheelMovement(double input)
    {

        /*double flywheelDeltaTime = flywheelTime.seconds() - prevFlywheelSnapshot;
        double targetRps = input * (flywheel.getMotorType().getMaxRPM() / 60);
        currentSnapshotRps = (flywheel.getCurrentPosition() / flywheel.getMotorType().getTicksPerRev()) / flywheelDeltaTime) - lastSnapshotRps;
        double error = targetRps - currentSnapshotRps;

        double P = FLYWHEEL_KP * error;
        double D = FLYWHEEL_KD * (error - oldFlywheelError);
        flywheelI += FLYWHEEL_KI * error;

        flywheel.setPower(P + flywheelI + D);
        lastSnapshotRps = currentSnapshotRps;
        oldFlywheelError = error;*/
        flywheel.setPower(input);
    }

    public double GetCurrentSnapshotRps() {return currentSnapshotRps;}

    public double ReadHue(ColorSensor sensor)
    {
        //Normalize RGB values to range [0, 1]
        double hue;
        float red = sensor.red() / 550f;
        float green = sensor.green() / 600f; //Sensor seems to have a bias towards green
        float blue = sensor.blue() / 550f;

        // Find the maximum and minimum values among RGB
        float maxColor = Math.max(red, Math.max(green, blue));
        float minColor = Math.min(red, Math.min(green, blue));
        float deltaColor = maxColor - minColor;

        // Calculate the hue
        if (deltaColor == 0) {hue = 0;}
        else if (maxColor == red) {hue = ((green - blue) / deltaColor) % 6;}
        else if (maxColor == green) {hue = ((blue - red) / deltaColor) + 2;}
        else {hue = ((red - green) / deltaColor) + 4;}

        hue *= 60;
        if (hue < 0) {hue += 360;}

        return hue;
    }

    // --- Intake state machine ---
    enum IntakeState
    {
        IDLE,
        EATING,
        SPINNING,
        FULL
    }

    public void UpdateIntake()
    {
        switch (intakeState)
        {
            case EATING:

                if (indexerSpinning) {break;}

                intake.setPower(-0.55);

                double distance = ballDistance.getDistance(DistanceUnit.CM);
                double hue = ReadHue(ballColor);

                if (distance < 16)
                {
                    ballEntered = true;
                    //distance = 16 => closest to color sensor => highest hues
                    if (hue >= 125 + (distance / 3) && hue <= 145 + (distance / 3))
                    {Global.indexerSlots[Global.currentSlot] = 2;} //gren
                    else
                    {Global.indexerSlots[Global.currentSlot] = 1;} //puple
                }
                //Wait for ball to enter indexer before stopping intake
                if (ballEntered && distance > 16)
                {
                    intakeTimer.reset();
                    ballEntered = false;
                    intakeState = IntakeState.SPINNING;
                }

                break;

            case SPINNING:
                //Wait for ball to nestle in indexer before spinning it
                if (intakeTimer.milliseconds() <= 300) {break;}

                intake.setPower(0);
                StartIndexerSpin();
                //Prevent activation of intake if indexer is full
                boolean indexerFull = true;
                for (int i = 0; i < 3; i++)
                {
                    if (Global.indexerSlots[i] == 0)
                    {
                        indexerFull = false;
                        break;
                    }
                }
                if (indexerFull) {intakeState = IntakeState.FULL;}
                else {intakeState = IntakeState.EATING;}

                break;

            case FULL:

                FlywheelMovement(-0.8);
                break;

            case IDLE:
            default:

                break;
        }
    }

    // --- Shooting state machine ---
    enum ShootingState
    {
        IDLE,
        TRANSFER_UP,
        TRANSFER_DOWN,
        SPINNING,
    }

    public void UpdateShooting()
    {
        switch (shootingState)
        {
            case TRANSFER_UP:

                boolean fallbackLaunch = true;
                for (int i = 0; i < 3; i++)
                {
                    if (Global.indexerSlots[i] == Global.pattern[Global.currentPatternSlot])
                    {fallbackLaunch = false; break;}
                }

                if (indexerSpinning) {break;}

                if (Global.indexerSlots[Global.currentSlot] != Global.pattern[Global.currentPatternSlot] && !fallbackLaunch)
                {shootingState = ShootingState.SPINNING; break;}

                transfer.setPosition(0.725);

                transferTimer.reset();

                shootingState = ShootingState.TRANSFER_DOWN;

                Global.indexerSlots[Global.currentSlot] = 0;
                Global.currentPatternSlot++;
                if (Global.currentPatternSlot >= 3) {Global.currentPatternSlot = 0;}

                break;

            case TRANSFER_DOWN:

                if (transferTimer.milliseconds() <= 500) {break;}

                transfer.setPosition(0.325);
                transferTimer.reset();
                shootingState = ShootingState.SPINNING;
                break;

            case SPINNING:

                if (transferTimer.milliseconds() <= 400) {break;}

                StartIndexerSpin();
                //Switch back to intake mode if indexer is empty
                boolean indexerEmpty = true;
                for (int i = 0; i < 3; i++)
                {
                    if (Global.indexerSlots[i] != 0) {indexerEmpty = false; break;}
                }
                if (indexerEmpty) {SwitchMode();}
                else {shootingState = ShootingState.TRANSFER_UP;}
                break;

            case IDLE:
            default:

                break;
        }
    }

    // --- Indexer state machine ---
    public void StartIndexerSpin()
    {
        targetIndexerAngle += INDEXER_TURN;
        if (targetIndexerAngle >= INDEXER_TURN * 3)
        {
            targetIndexerAngle -= INDEXER_TURN * 3;
            mustReachZero = true;
        }
        indexerSpinning = true;
    }

    public double GetIndexerAngle() {return adjustedAngle;}
    public boolean GetIndexerState() {return indexerSpinning;}
    public boolean GetMustReachZero() {return mustReachZero;}
    public double GetTargetIndexerAngle() {return targetIndexerAngle;}

    public double GetIndexerPower() {return indexerInput;}

    public void UpdateIndexerSpin()
    {
        adjustedAngle = Range.scale(indexerAngle.getVoltage(), 0, indexerAngle.getMaxVoltage(), 0, INDEXER_TURN*3);

        double error;
        if (mustReachZero) {error = adjustedAngle;}
        else {error = targetIndexerAngle - adjustedAngle;}

        indexerP = INDEXER_KP * error;
        indexerD = INDEXER_KD * (error - oldIndexerError);
        if (!(indexerInput >= 1 && error > 0)) {indexerI += INDEXER_KI * error;} //ceiling for I

        indexerInput = (indexerP+indexerI+indexerD) /INDEXER_TURN/3;

        indexer.setPower(-indexerInput);
        oldIndexerError = error;

        //Technically angle isn't fully corrected yet but it's close so we can afford starting intake
        if (adjustedAngle >= targetIndexerAngle - INDEXER_TURN*0.05 && !mustReachZero && indexerSpinning)
        {
            Global.currentSlot++;
            if (Global.currentSlot >= 3) {Global.currentSlot = 0;}
            indexerSpinning = false;
        }
        //Force wrap around so indexer keeps the same direction
        else if (mustReachZero && adjustedAngle < INDEXER_TURN / 8)
        {mustReachZero = false;}
    }
}
