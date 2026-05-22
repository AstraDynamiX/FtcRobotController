package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

class AutoStep
{
    PathChain path;
    Runnable action;
    double speed;
    double delay;
    double actionDelay;

    public AutoStep(PathChain path, Runnable action, double speed, double delay, double actionDelay)
    {
        this.path = path;
        this.action = action;
        this.speed = speed;
        this.delay = delay;
        this.actionDelay = actionDelay;
    }
}

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_Revamped")
public class Autonomous extends OpMode
{
    private final double SHOOT_DELAY = 650; //Wait for turret to adjust
    private final double SHOOT_TIME = 2600; //Wait for shooting before starting path
    private final double INTAKE_SPEED = 0.5;

    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //Blue close
    private final Pose startCloseBlue = new Pose(25.083, 131.242, Math.toRadians(143.8));
    private final Pose shootCloseBlue = new Pose(55.786, 87.350, Math.toRadians(140));
    private final Pose intake1StartCloseBlue = new Pose(52.297, 87.270, Math.toRadians(180));
    private final Pose intake1EndCloseBlue = new Pose(19.299, 87.270, Math.toRadians(180));
    private final Pose intake2StartCloseBlue = new Pose(50.081, 63.703, Math.toRadians(180));
    private final Pose intake2EndCloseBlue = new Pose(17.702, 63.594, Math.toRadians(180));
    private final Pose intake3StartCloseBlue = new Pose(55.865, 39.243, Math.toRadians(180));
    private final Pose intake3EndCloseBlue = new Pose(20.541, 39.243, Math.toRadians(180));
    private final Pose endCloseBlue = new Pose(24.919, 79.978, Math.toRadians(-135));

    //Blue far
    private final Pose startFarBlue = new Pose(56,8, Math.toRadians(90));
    private final Pose shootFarBlue = new Pose(57.730,13.514, Math.toRadians(110));
    private final Pose intake1StartFarBlue = new Pose(55.865, 37.243, Math.toRadians(180));
    private final Pose intake1EndFarBlue = new Pose(20.541, 37.243, Math.toRadians(180));
    private final Pose intake2StartFarBlue = new Pose(20.919, 44.838, Math.toRadians(-95));
    private final Pose intake2EndFarBlue = new Pose(18, 17.459, Math.toRadians(-95));
    private final Pose endFarBlue = new Pose(38.108, 26.297, Math.toRadians(-150));

    //Red close
    private final Pose startCloseRed = new Pose(118.917, 131.242, Math.toRadians(36.2));
    private final Pose shootCloseRed = new Pose(88.214, 87.350, Math.toRadians(45));
    private final Pose intake1StartCloseRed = new Pose(91.703, 87.270, Math.toRadians(360));
    private final Pose intake1EndCloseRed = new Pose(124.701, 87.270, Math.toRadians(360));
    private final Pose intake2StartCloseRed = new Pose(93.919, 63.703, Math.toRadians(360));
    private final Pose intake2EndCloseRed = new Pose(126.298, 63.594, Math.toRadians(360));
    private final Pose intake3StartCloseRed = new Pose(0, 39.243, Math.toRadians(180)); //TODO
    private final Pose intake3EndCloseRed = new Pose(0, 39.243, Math.toRadians(180)); //TODO
    private final Pose endCloseRed = new Pose(119.081, 79.978, Math.toRadians(250));

    //Red far
    private final Pose startFarRed = new Pose(88, 8, Math.toRadians(90));
    private final Pose shootFarRed = new Pose(86.486, 18.054, Math.toRadians(70));
    private final Pose intake1StartFarRed = new Pose(93.135, 37.243, Math.toRadians(0));
    private final Pose intake1EndFarRed = new Pose(126.459, 37.243, Math.toRadians(0));
    private final Pose intake2StartFarRed = new Pose(133.757, 38.514, Math.toRadians(0));
    private final Pose intake2EndFarRed = new Pose(135.432, 8.757, Math.toRadians(0));
    private final Pose endFarRed = new Pose(38.108, 26.297, Math.toRadians(275));


    List<AutoStep> steps = new ArrayList<>();
    int stepIndex = 0;
    private boolean stepStarted = false;
    private boolean pathStarted = false;
    private boolean actionStarted = false;

    //Trajectory selection booleans
    boolean redAlliance = false;
    boolean closeTrajectory = true;
    boolean confirmed = false;
    boolean camAdjustment = true;


    @Override
    public void init()
    {
        opModeTimer.reset();
        follower = Constants.createFollower(hardwareMap);

        steps.clear();
        stepIndex = 0;
        stepStarted = false;
        pathStarted = false;
        actionStarted = false;
    }

    @Override
    public void init_loop()
    {
        if (confirmed)
        {
            telemetry.addData("CONFIRMED", "");
            telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");
            telemetry.addData("TRAJECTORY", (closeTrajectory) ? "close" : "far");
        }
        else
        {
            if (gamepad1.bWasPressed())
            {redAlliance = !redAlliance;}

            if (gamepad1.xWasPressed())
            {closeTrajectory = !closeTrajectory;}

            if (gamepad1.a)
            {
                confirmed = true;
                LaunchBoard.init(hardwareMap, redAlliance);
            }

            telemetry.addData("", "() - change alliance, [] - change trajectory, X - confirm");
            telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");
            telemetry.addData("TRAJECTORY", (closeTrajectory) ? "close" : "far");

        }
    }

    @Override
    public void start()
    {
        opModeTimer.reset();
        LaunchBoard.start();

        if (redAlliance)
        {
            if (closeTrajectory)
            {
                follower.setPose(startCloseRed);

                AddStep(startCloseRed, shootCloseRed, () -> LaunchBoard.Rev(), 1,  0, 0);
                AddStep(shootCloseRed, intake1StartCloseRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake1StartCloseRed, intake1EndCloseRed, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake1EndCloseRed, shootCloseRed, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseRed, intake2StartCloseRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake2StartCloseRed, intake2EndCloseRed, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake2EndCloseRed, shootCloseRed, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseRed, startCloseRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                /*AddStep(shootCloseRed, intake3StartCloseRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake3StartCloseRed, intake3EndCloseRed, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake3EndCloseRed, shootCloseRed, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseRed, endCloseRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);*/
            }
            else
            {
                follower.setPose(startFarRed);

                AddStep(startFarRed, shootFarRed, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootFarRed, intake1StartFarRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake1StartFarRed, intake1EndFarRed, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake1EndFarRed, shootFarRed, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootFarRed, intake2StartFarRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake2StartFarRed, intake2EndFarRed, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake2EndFarRed, shootFarRed, () -> LaunchBoard.Rev(), 0.8, 0, 0);
                AddStep(shootFarRed, endFarRed, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            }
        }
        else
        {
            if (closeTrajectory)
            {
                follower.setPose(startCloseBlue);

                AddStep(startCloseBlue, shootCloseBlue, () -> LaunchBoard.Rev(), 1,  0, 0);
                AddStep(shootCloseBlue, intake1StartCloseBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake1StartCloseBlue, intake1EndCloseBlue, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake1EndCloseBlue, shootCloseBlue, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseBlue, intake2StartCloseBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake2StartCloseBlue, intake2EndCloseBlue, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake2EndCloseBlue, shootCloseBlue, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseBlue, startCloseBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                /*AddStep(shootCloseBlue, intake3StartCloseBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake3StartCloseBlue, intake3EndCloseBlue, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake3EndCloseBlue, shootCloseBlue, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootCloseBlue, endCloseBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);*/
            }
            else
            {
                follower.setPose(startFarBlue);

                AddStep(startFarBlue, shootFarBlue, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootFarBlue, intake1StartFarBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake1StartFarBlue, intake1EndFarBlue, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake1EndFarBlue, shootFarBlue, () -> LaunchBoard.Rev(), 1, 0, 0);
                AddStep(shootFarBlue, intake2StartFarBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
                AddStep(intake2StartFarBlue, intake2EndFarBlue, () -> LaunchBoard.Intake(), INTAKE_SPEED, 0, 0);
                AddStep(intake2EndFarBlue, shootFarBlue, () -> LaunchBoard.Rev(), 0.8, 0, 0);
                AddStep(shootFarBlue, endFarBlue, () -> LaunchBoard.Shoot(), 1, SHOOT_TIME, SHOOT_DELAY);
            }
        }
    }

    @Override
    public void loop()
    {
        follower.update();
        RunScheduler();

        if (camAdjustment)
        {
            LaunchBoard.UpdateLaunch(true, 1);
            LaunchBoard.TurretMovement();
        }
        else
        {
            LaunchBoard.UpdateLaunch(false,1);
            LaunchBoard.TurretLockPosition(0);
        }

        if (opModeTimer.seconds() > 30) {requestOpModeStop();}
    }

    @Override
    public void stop()
    {
        LaunchBoard.stop();
    }


    void AddStep(Pose start, Pose end, Runnable action, double speed, double delay, double actionDelay)
    {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();

        steps.add(new AutoStep(path, action, speed, delay, actionDelay));
    }

    public void RunScheduler() {

        if (stepIndex >= steps.size())
        {
            LaunchBoard.Idle();
            return;
        }
        AutoStep step = steps.get(stepIndex);

        //Reset timer after each new step
        if (!stepStarted)
        {
            timer.reset();
            stepStarted = true;
        }

        if (!pathStarted && timer.milliseconds() > step.delay)
        {
            pathStarted = true;
            follower.setMaxPower(step.speed);
            follower.followPath(step.path, true);
        }

        //Run action after specified time
        if (!actionStarted && timer.milliseconds() > step.actionDelay)
        {
            if (step.action != null) step.action.run();
            actionStarted = true;
        }

        //Run next path after the previous one is finished
        if (!follower.isBusy() && pathStarted && actionStarted)
        {
            stepIndex++;
            stepStarted = false;
            pathStarted = false;
            actionStarted = false;
        }
    }
}
