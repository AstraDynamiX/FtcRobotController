package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    @Override
    public void init_loop()
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        double distance = 0;

        for (AprilTagDetection detection : currentDetections)
        {
            distance = detection.robotPose.getPosition().z;
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
        telemetry.addData("Distance", distance);
    }

    @Override
    public void loop() {}
}
