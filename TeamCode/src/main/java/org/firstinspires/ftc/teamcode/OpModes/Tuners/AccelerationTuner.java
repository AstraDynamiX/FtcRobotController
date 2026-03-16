package org.firstinspires.ftc.teamcode.OpModes.Tuners;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(group = "tests")
public class AccelerationTuner extends OpMode {

    double maxSpeed = 81;
    double acceleration = 0;
    ElapsedTime timer = new ElapsedTime();
    GoBildaPinpointDriver pinpoint;

    private Follower follower;
    public static Pose startingPose;

    LaunchBoard LaunchBoard = new LaunchBoard();


    @Override

    public void init() {
        LaunchBoard.init(hardwareMap, true);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(4.92, 1.81, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    @Override

    public void start() {
        LaunchBoard.start();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        timer.reset();

        while (velX < maxSpeed) {
            follower.setTeleOpDrive(0.96, 0, 0, 1);
            if (acceleration == 0) {
                acceleration = maxSpeed / timer.seconds();
            }
            telemetry.addData("Acceleration = ", acceleration);
        }
    }
}