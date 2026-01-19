package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }

    public double GetAprilTag(int id)
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        double distance = -1;

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.id != id) {continue;}
            if (detection.ftcPose != null)
            {distance = detection.ftcPose.range;}
        }
        return distance;
    }

    public double ReadHue(ColorSensor sensor)
    {
        //Normalize RGB values to range [0, 1]
        double hue;
        float red = sensor.red() / 550f;
        float green = sensor.green() / 600f; //Sensor seems to have a bias towards green
        float blue = sensor.blue() / 550f;

        // Find the maximum and minimum values among RGB
        float maxColor = Math.max(red, Math.max(green, blue));
        float minColor = Math.min(red, Math.min(green, blue));
        float deltaColor = maxColor - minColor;

        // Calculate the hue
        if (deltaColor == 0) {hue = 0;}
        else if (maxColor == red) {hue = ((green - blue) / deltaColor) % 6;}
        else if (maxColor == green) {hue = ((blue - red) / deltaColor) + 2;}
        else {hue = ((red - green) / deltaColor) + 4;}

        hue *= 60;
        if (hue < 0) {hue += 360;}

        return hue;
    }

}

