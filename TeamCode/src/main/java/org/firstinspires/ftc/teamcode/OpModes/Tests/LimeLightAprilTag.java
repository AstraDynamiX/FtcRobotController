package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class LimeLightAprilTag extends OpMode {
    private Limelight3A limelight;
    DcMotor turret;

    // CAMERA OFFSET - tx când e centrat pe goal
    private static final double CAMERA_OFFSET_TX = 7.60;

    // Control simplu și rapid
    private final double kP = 0.055;              // Proportional gain
    private final double maxPower = 0.5;          // Max power pentru viteză
    private final double deadzone = 1.0;          // ±1° toleranță

    // Limite turretă
    private static final double DEGREES_PER_TICK = 360.0 / (537.7);
    private static final double MAX_CCW = -90;
    private static final double MAX_CW = 180;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // IMPORTANT - nu folosim RUN_TO_POSITION

        // Dacă se mișcă în direcția greșită, decomentează:
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);

        telemetry.addLine("Continuous Tracking Ready");
        telemetry.addData("Camera offset", CAMERA_OFFSET_TX);
        telemetry.update();
    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        // Verifică dacă vede AprilTag
        if (!limelight.getLatestResult().isValid()) {
            turret.setPower(0);
            telemetry.addLine("❌ No AprilTag");
            telemetry.update();
            return;
        }

        double rawTx = limelight.getLatestResult().getTx();
        double error = rawTx - CAMERA_OFFSET_TX;
        double currentPosDegrees = turret.getCurrentPosition() * DEGREES_PER_TICK;

        telemetry.addData("Raw tx", "%.2f", rawTx);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Turret pos", "%.1f°", currentPosDegrees);

        // Dacă e în deadzone, stop
        if (Math.abs(error) < deadzone) {
            turret.setPower(0);
            telemetry.addLine("✓ CENTERED");
            telemetry.update();
            return;
        }

        // Calculează power proporțional cu error
        // Error pozitiv (tag în dreapta) → rotește dreapta
        // Error negativ (tag în stânga) → rotește stânga
        double power = error * kP;
        power = Range.clip(power, -maxPower, maxPower);

        // Verifică limite - CORECTATĂ
        if ((currentPosDegrees <= MAX_CCW && power > 0) ||   // La -90° și vrea să meargă mai negativ
                (currentPosDegrees >= MAX_CW && power < 0)) {    // La 180° și vrea să meargă mai pozitiv
            turret.setPower(0);
            telemetry.addLine("⚠️ At limit");
            telemetry.addData("Which limit?", currentPosDegrees <= MAX_CCW ? "CCW (-90°)" : "CW (+180°)");
        } else {
            turret.setPower(-power);

            String direction = error > 0 ? "<- LEFT" : "RIGHT ->";
            telemetry.addData("Tracking", direction);
            telemetry.addData("Power", "%.2f", power);
        }


    }
}