package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;


@TeleOp(group = "tests")
public class LimeLightAprilTagRecognition extends OpMode
{
    private Limelight3A limelight;

    private static final double BEARING_OFFSET = 0;
    private static final double CALIBRATION_CONSTANT = 40 * sqrt(3.62);

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);

        telemetry.addLine("Continuous Tracking Ready");
        telemetry.addData("Camera offset", BEARING_OFFSET);
        telemetry.update();
    }

    @Override
    public void start()
    {
        limelight.start();
    }

    @Override
    public void loop()
    {
        LLResult result = limelight.getLatestResult();

        if (!result.isValid())
        {
            telemetry.addLine("❌ No AprilTag");
            return;
        }
        double ta = result.getTa();
        double distanceInches = CALIBRATION_CONSTANT / sqrt(ta);
        telemetry.addData("Distance", "%.1f\"", distanceInches);

        double bearing = limelight.getLatestResult().getTx() - BEARING_OFFSET;
        telemetry.addData("Bearing", "%.2f", bearing);
    }

    @Override
    public void stop()
    {
        limelight.stop();
        limelight.close();
    }
}