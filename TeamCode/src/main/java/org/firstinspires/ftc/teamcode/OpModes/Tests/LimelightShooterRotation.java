package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Limelight3A Shooter Aim (SDK)")
public class LimelightShooterRotation extends OpMode {

    DcMotor shooterYaw;
    Limelight3A limelight;

    double kP = 0.02;
    double deadzone = 1.0; // grade

    @Override
    public void init() {

        shooterYaw = hardwareMap.get(DcMotor.class, "shooterYaw");
        shooterYaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // DEFINIRE LIMELIGHT 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // pipeline-ul de AprilTag (setat din UI Limelight)
        limelight.pipelineSwitch(8);

        telemetry.addLine("Limelight 3A READY");
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx(); // offset stânga/dreapta

            double power = tx * kP;

            if (Math.abs(tx) < deadzone) {
                power = 0;
            }

            shooterYaw.setPower(power);

            telemetry.addData("Target", "YES");
            telemetry.addData("tx", tx);
            telemetry.addData("Motor power", power);

        } else {
            shooterYaw.setPower(0);
            telemetry.addData("Target", "NO");
        }

        telemetry.update();
    }
}
