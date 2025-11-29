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
    private final double FLYWHEEL_RPT = 6000 / 60 / 20; //Rotations per function call (50ms)
    private final double FLYWHEEL_KP = 0;
    private final double FLYWHEEL_KI = 0;
    private final double FLYWHEEL_KD = 0;

    private DcMotor intake;
    private DcMotor flywheel;
    private Servo transfer;
    private CRServo indexer;

    private ColorSensor ballColor;
    private DistanceSensor ballDistance;
    private AnalogInput indexerAngle;


    private boolean isShooting = false;
    IntakeState intakeState = IntakeState.IDLE;
    ShootingState shootingState = ShootingState.IDLE;
    ElapsedTime transferTimer = new ElapsedTime();
    private IndexerState indexerState = IndexerState.IDLE;
    private double targetIndexerAngle = -2; //Offset


    private double oldFlywheelError = 0;
    private double lastTickRotations = 0;
    private double flywheelI = 0;


    public void init(HardwareMap hwMap)
    {
        Global.indexerSlots = new double[]{0, 0, 0};
        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        transfer = hwMap.get(Servo.class, "transfer");
        indexer = hwMap.get(CRServo.class, "indexer");

        ballColor = hwMap.get(ColorSensor.class, "ballColor");
        ballDistance = hwMap.get(DistanceSensor.class, "ballDistance");
        indexerAngle = hwMap.get(AnalogInput.class, "indexerAngle");
        //Default run mode
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SwitchMode()
    {
        isShooting = !isShooting;
        if (isShooting)
        {
            targetIndexerAngle += 60;
            if (intakeState != IntakeState.FULL) {intakeState = IntakeState.IDLE;}
            shootingState = ShootingState.TRANSFER_UP;
        }
        else
        {
            targetIndexerAngle -= 60;
            if (intakeState != IntakeState.FULL) {intakeState = IntakeState.EATING;}
            shootingState = ShootingState.IDLE;
        }
    }

    //Separate because flywheel needs time to rev
    public void FlywheelMovement(double input)
    {
        /*double targetRpt = input * FLYWHEEL_RPT;
        double currentTickRotations =  flywheel.getCurrentPosition() / flywheel.getMotorType().getTicksPerRev() - lastTickRotations;
        double error = targetRpt - currentTickRotations;

        double P = FLYWHEEL_KP * error;
        double D = FLYWHEEL_KD * (error - oldFlywheelError);
        flywheelI += FLYWHEEL_KI * error;

        flywheel.setPower(P + flywheelI + D);
        lastTickRotations = currentTickRotations;
        oldFlywheelError = error;*/
        flywheel.setPower(input);
    }

    public void TransferMovement(double input) {transfer.setPosition(input);}

    public void IntakeMovement(double input) {intake.setPower(input);}

    public double ReadHue(ColorSensor sensor)
    {
        //Normalize RGB values to range [0, 1]
        double hue;
        float red = sensor.red() / 255f;
        float green = sensor.green() / 255f;
        float blue = sensor.blue() / 255f;

        // Find the maximum and minimum values among RGB
        float maxColor = Math.max(red, Math.max(green, blue));
        float minColor = Math.min(red, Math.min(green, blue));
        float deltaColor = maxColor - minColor;

        // Calculate the hue
        if (deltaColor == 0) {hue = 0;}
        else if (maxColor == red) {hue = ((green - blue) / deltaColor) % 6;}
        else if (maxColor == green) {hue = ((blue - red) / deltaColor) + 2;}
        else {hue = ((red - green) / deltaColor) + 4;}

        //
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

                if (indexerState == IndexerState.SPINNING) {break;}

                IntakeMovement(-0.6);
                if (ballDistance.getDistance(DistanceUnit.CM) < 10)
                {
                    if (ReadHue(ballColor) >= 130 && ReadHue(ballColor) <= 150)
                    {Global.indexerSlots[Global.currentSlot] = 2;} //gren
                    else /*if (ReadHue(ballColor) >= 270 && ReadHue(ballColor) <= 285)*/
                    {Global.indexerSlots[Global.currentSlot] = 1;} //puple

                    intakeState = IntakeState.SPINNING;
                }
                break;

            case SPINNING:

                IntakeMovement(0);
                StartIndexerSpin();
                //Prevent activation of intake if indexer is full
                boolean indexerFull = true;
                for (int i = 0; i < 3; i++)
                {
                    if (Global.indexerSlots[i] == 0) {indexerFull = false; break;}
                }
                if (indexerFull) {intakeState = IntakeState.FULL;}
                else {intakeState = IntakeState.EATING;}
                break;

            case FULL:
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

                if (indexerState == IndexerState.SPINNING) {break;}
                if (Global.indexerSlots[Global.currentSlot] != Global.pattern[Global.currentPatternSlot])
                {shootingState = ShootingState.SPINNING; break;}

                TransferMovement(0.35);
                transferTimer.reset();

                Global.indexerSlots[Global.currentSlot] = 0;
                shootingState = ShootingState.TRANSFER_DOWN;
                break;

            case TRANSFER_DOWN:

                if (transferTimer.milliseconds() <= 300) {break;}
                TransferMovement(0.1);
                transferTimer.reset();
                shootingState = ShootingState.SPINNING;
                break;

            case SPINNING:

                if (transferTimer.milliseconds() <= 300) {break;}
                StartIndexerSpin();
                //Switch back to intake mode if indexer is empty
                boolean indexerEmpty = true;
                for (int i = 0; i < 3; i++)
                {
                    if (Global.indexerSlots[i] != 0) {indexerEmpty = false; break;}
                }
                if (indexerEmpty) {shootingState = ShootingState.IDLE;}
                else {shootingState = ShootingState.TRANSFER_UP;}
                break;

            case IDLE:
            default:

                break;
        }
    }

    // --- Indexer state machine ---
    enum IndexerState
    {
        IDLE,
        SPINNING
    }

    public void StartIndexerSpin()
    {
        indexerState = IndexerState.SPINNING;
        targetIndexerAngle += 120;
        if (targetIndexerAngle == 480) targetIndexerAngle = 120;
    }

    public double GetIndexerAngle() {return Range.scale(indexerAngle.getVoltage(), 0, indexerAngle.getMaxVoltage(), 0, 360);}

    public void UpdateIndexerSpin()
    {
        switch (indexerState) {
            case SPINNING:
                indexer.setPower(-0.6);
                double adjustedTarget = Range.scale(indexerAngle.getVoltage(), 0, indexerAngle.getMaxVoltage(), 0, 360);
                if (adjustedTarget >= targetIndexerAngle)
                {
                    Global.currentSlot++;
                    if (Global.currentSlot >= 3) {Global.currentSlot = 0;}
                    indexerState = IndexerState.IDLE;
                }
                break;

            case IDLE:
            default:
                indexer.setPower(0);
                break;
        }
    }
}
