package org.firstinspires.ftc.teamcode.OpModes.Tests;

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
    CameraBoard CamBoard = new CameraBoard();

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init()
    {
        CamBoard.init(hardwareMap);
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
            distance = detection.ftcPose.range;
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
        telemetry.addData("Distance", distance);
        if (CamBoard.GetAprilTag(24) != -1)
        {telemetry.addData("APRIL TAG DISTANCE", CamBoard.GetAprilTag(24));}
    }

    @Override
    public void loop() {}
}
