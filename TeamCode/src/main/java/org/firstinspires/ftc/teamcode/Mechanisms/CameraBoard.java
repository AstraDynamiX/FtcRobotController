package org.firstinspires.ftc.teamcode.Mechanisms;

import android.util.Size;

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
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public double GetAprilTag(int id, String dimension)
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        double distance = 400; //Impossible value to tell if data has actually been found

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.id != id) {continue;}
            if (detection.ftcPose != null)
            {
                switch (dimension)
                {
                    case "range": distance = detection.ftcPose.range; break;
                    case "bearing": distance = detection.ftcPose.bearing; break;
                    case "yaw": distance = detection.ftcPose.yaw; break;
                }
            }
        }
        return distance;
    }

    //Returns all dimensions in a list
    public double[] GetAprilTag(int id)
    {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        double[] info = {400, 400, 400}; //Impossible values for each to tell if data has actually been found

        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.id != id) {continue;}
            if (detection.ftcPose != null)
            {
                info[0] = detection.ftcPose.range;
                info[1] = detection.ftcPose.bearing;
                info[2] = detection.ftcPose.yaw;
            }
        }
        return info;
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

