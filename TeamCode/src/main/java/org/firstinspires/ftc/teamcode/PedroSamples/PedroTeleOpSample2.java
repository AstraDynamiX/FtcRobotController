package org.firstinspires.ftc.teamcode.PedroSamples;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(group="zPedroSamples")
public class PedroTeleOpSample2 extends OpMode
{
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean automatedDrive;
    private boolean fieldCentricDrive = false;


    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start()
    {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop()
    {
        follower.update();
        telemetryM.update();

        if (!automatedDrive)
        {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y / (gamepad1.left_trigger+1),
                    -gamepad1.left_stick_x / (gamepad1.left_trigger+1),
                    -gamepad1.right_stick_x / (gamepad1.left_trigger+1),
                    fieldCentricDrive
            );
        }

        if (gamepad1.shareWasPressed())
        {fieldCentricDrive = !fieldCentricDrive;}

        //Automated PathFollowing
        if (gamepad1.aWasPressed())
        {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy()))
        {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

}