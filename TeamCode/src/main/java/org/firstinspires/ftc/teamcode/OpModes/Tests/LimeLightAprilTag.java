package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class LimeLightAprilTag extends OpMode {
    private Limelight3A limelight;
    DcMotor turretMotor;

    private final double kP=0.02;
    private final double kD = 0.003;
    private final  double maxPower = 0.3;
    private final double deadzone = 1.0;

    private double lastTx = 0.0;

    @Override
    public void init() {
        //tureta
        turretMotor= hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        telemetry.addLine("AprilTag Follow Ready");
        telemetry.update();
    }
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        double tx = limelight.getLatestResult().getTx();

        if(Math.abs(tx) < deadzone) {
            turretMotor.setPower(0.0);
            lastTx = tx;
            return;
        }
        double error = tx;
        double derivative = error - lastTx;
        double power = (error * kP) + (derivative * kD);
        power = Range.clip(power, -maxPower, maxPower);

        turretMotor.setPower(-power);

        lastTx = error;

        telemetry.addData("tx", tx);
        telemetry.addData("power", power);
        telemetry.update();

    }
}
