package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Mechanisms.CameraBoard;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(group = "tests")
public class CameraAprilTagRecognition extends OpMode
{
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init()
    {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    @Override
    public void init_loop()
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        double distance = 0;
        double bearing = 0;

        for (AprilTagDetection detection : currentDetections)
        {
            distance = detection.ftcPose.range;
            bearing = detection.ftcPose.bearing;
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
        telemetry.addData("Distance", distance);
        telemetry.addData("Bearing", bearing);
    }

    @Override
    public void loop() {}
}
