package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class LaunchBoard
{
    private final double FLYWHEEL_RPT = 6000 / 60 / 20; //Rotations per function call
    private final double FLYWHEEL_KP = 0;
    private final double FLYWHEEL_KI = 0;
    private final double FLYWHEEL_KD = 0;

    private DcMotor intake;
    private DcMotor flywheel;
    private Servo transfer;
    private CRServo indexer;

    private VoltageSensor battery;
    private AnalogInput indexerAngle;

    private double targetIndexerAngle = 120;
    private IndexerState indexerState = IndexerState.IDLE;

    private double oldFlywheelError = 0;
    private double lastTickRotations = 0;


    public void init(HardwareMap hwMap)
    {
        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        transfer = hwMap.get(Servo.class, "transfer");
        indexer = hwMap.get(CRServo.class, "indexer");

        battery = hwMap.get(VoltageSensor.class, "voltageSensor");
        indexerAngle = hwMap.get(AnalogInput.class, "indexerAngle");
        //Default run mode
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Separate because flywheel needs time to rev
    public void FlywheelMovement(double input)
    {
        double targetRpt = input * FLYWHEEL_RPT;
        double currentTickRotations =  flywheel.getCurrentPosition() / flywheel.getMotorType().getTicksPerRev() - lastTickRotations;
        double error = targetRpt - currentTickRotations;

        double P = FLYWHEEL_KP * error;
        double D = FLYWHEEL_KD * (error - oldFlywheelError);
        double I = 0;
        I += FLYWHEEL_KI * error;

        flywheel.setPower(P + I + D);
        lastTickRotations = currentTickRotations;
        oldFlywheelError = error;
    }

    public void TransferMovement(double input) {transfer.setPosition(input);}

    public void IntakeMovement(double input) {intake.setPower(input);}


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
    public double GetTargetIndexerAngle() {return targetIndexerAngle;}

    public void UpdateIndexerSpin()
    {
        switch (indexerState) {
            case SPINNING:
                indexer.setPower(-0.6);
                double adjustedTarget = Range.scale(indexerAngle.getVoltage(), 0, indexerAngle.getMaxVoltage(), 0, 360);
                if (adjustedTarget >= targetIndexerAngle) {
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
