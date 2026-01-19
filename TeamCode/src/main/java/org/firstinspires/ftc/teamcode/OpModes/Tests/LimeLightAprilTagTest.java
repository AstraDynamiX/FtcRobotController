package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous
public class LimeLightAprilTagTest extends OpMode {
    private Limelight3A limelight;
    private double distance;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);//april tag nr 20

    }
    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Calculated Distance: ", distance);
            telemetry.addData("Target x: ", llResult.getTx());
            telemetry.addData("ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.getOrientation().getYaw());

        }
    }
    public double getDistanceFromTag(double ta){
        double scale = 196.9548;
        double distance = (scale / ta);
        return distance;
    }

}
