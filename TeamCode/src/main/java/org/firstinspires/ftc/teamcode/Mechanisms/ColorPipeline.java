package org.firstinspires.ftc.teamcode.Mechanisms;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class ColorPipeline extends OpenCvPipeline {

    public String detectedColor = "NONE";
    private Mat hsv = new Mat();
    private Mat maskGreen = new Mat();
    private Mat maskPurple = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Verde
        Core.inRange(hsv, new Scalar(35,50,50), new Scalar(90,255,255), maskGreen);
        // Mov
        Core.inRange(hsv, new Scalar(130,50,50), new Scalar(160,255,255), maskPurple);

        double greenCount = Core.countNonZero(maskGreen);
        double purpleCount = Core.countNonZero(maskPurple);

        if (greenCount > purpleCount && greenCount > 3000)
            detectedColor = "GREEN";
        else if (purpleCount > greenCount && purpleCount > 3000)
            detectedColor = "PURPLE";
        else
            detectedColor = "NONE";

        // Overlay pentru debugging
        Imgproc.putText(input, "Color: " + detectedColor, new Point(20,40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0,255,255), 2);

        return input;
    }
}
