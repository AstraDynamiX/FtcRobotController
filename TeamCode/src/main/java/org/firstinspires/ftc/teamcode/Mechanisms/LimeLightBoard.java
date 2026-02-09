package org.firstinspires.ftc.teamcode.Mechanisms;

import static java.lang.Math.sqrt;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LimeLightBoard
{
    private Limelight3A limelight;

    private final double BEARING_OFFSET = -3;
    private final double CALIBRATION_CONSTANT = 40 * sqrt(3.62);

    public void init(HardwareMap hwMap, int id)
    {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(id);
    }

    public void start()
    {
        limelight.start();
    }

    public void stop()
    {
        limelight.stop();
        limelight.close();
    }

    public double GetAprilTag(String dimension)
    {
        if (limelight == null) {return 400;}
        LLResult detection = limelight.getLatestResult();
        double distance = 400; //Impossible value to tell if data has actually been found

        if (detection.isValid())
        {
            switch (dimension)
            {
                case "range":

                    double ta = detection.getTa();
                    distance = CALIBRATION_CONSTANT / sqrt(ta);
                    break;

                case "bearing":

                    distance = detection.getTx() - BEARING_OFFSET;
                    break;
            }
        }
        return distance;
    }

    //Returns all dimensions in a list
    public double[] GetAprilTag()
    {
        LLResult detection = limelight.getLatestResult();
        double[] info = {400, 400}; //Impossible values for each to tell if data has actually been found

        if (detection.isValid())
        {
            double ta = detection.getTa();
            info[0] = (CALIBRATION_CONSTANT / sqrt(ta));
            info[1] = detection.getTx() - BEARING_OFFSET;
        }
        return info;
    }
}