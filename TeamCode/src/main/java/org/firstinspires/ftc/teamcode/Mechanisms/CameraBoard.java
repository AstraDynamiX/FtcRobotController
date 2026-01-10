package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class CameraBoard
{
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public void init(HardwareMap hwMap)
    {
        //WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    public List<Double> GetAprilTag()
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        List<Double> info = new ArrayList<>();

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.ftcPose != null)
            {info.add(detection.ftcPose.range);}
        }
        return info;
    }



}

