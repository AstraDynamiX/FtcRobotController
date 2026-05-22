package org.firstinspires.ftc.teamcode.OpModes.Readers;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(group = "readers")
public class ZeroToFullVelocityReader extends OpMode
{
    public static double TOP_VELOCITY = 73; //Tune with PedroPathing
    private final double RECORD_FREQUENCY = 250; //Milliseconds

    private Follower follower;
    public static Pose startingPose;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime recordTimer = new ElapsedTime();

    private List<Double> records = new ArrayList<>();
    private int recordNumber = 1;
    double forwardVelocity;

    private boolean stop = false;
    private boolean finished = false;


    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetry.setAutoClear(false);
    }

    @Override
    public void start()
    {
        follower.startTeleopDrive(true);

        timer.reset();
        recordTimer.reset();
    }

    @Override
    public void loop()
    {
        if (finished) return;

        if (forwardVelocity >= TOP_VELOCITY || stop)
        {
            telemetry.addData("Time: ", timer.seconds());
            follower.setTeleOpDrive(0,0,0,true);
            finished = true;
        }

        else
        {
            forwardVelocity = follower.getVelocity().getXComponent();

            follower.setTeleOpDrive(0.9, 0, 0, true);

            if (recordTimer.milliseconds() > RECORD_FREQUENCY) {
                records.add(forwardVelocity);
                recordTimer.reset();

                String recordLine = String.format(Locale.US, "{%d. %.3f}", recordNumber, forwardVelocity);
                telemetry.addLine(recordLine);
                telemetry.update();
                recordNumber++;
            }

            if (gamepad1.b) {stop = true;}
        }

        follower.update();
    }

}
