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
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo transfer;
    private CRServo indexer;

    private VoltageSensor battery;
    private AnalogInput indexerAngle;

    private double targetIndexerAngle = 120;
    private IndexerState indexerState = IndexerState.IDLE;


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
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Separate because flywheel needs time to rev
    public void FlywheelMovement(double input)
    {
        double speed = 0.935 - Range.scale(battery.getVoltage(), 0, 13, 0, 0.935);
        flywheel.setPower(speed);
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
