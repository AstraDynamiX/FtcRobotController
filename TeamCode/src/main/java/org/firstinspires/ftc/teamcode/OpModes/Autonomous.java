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
    private final Pose startCloseBlue = new Pose(25.083168545284792,131.24159529402453,Math.toRadians(143.8));
    private final Pose shootCloseBlue = new Pose(55.78587124798748,87.34970340213259,Math.toRadians(140));
    private final Pose intake1StartCloseBlue = new Pose(52.29729670270271, 87.27027027027026, Math.toRadians(180));
    private final Pose intake1EndCloseBlue = new Pose(19.299384761501017, 87.27027027027029, Math.toRadians(180));
    private final Pose intake2StartCloseBlue = new Pose(50.08108108108108,63.7027027027027,Math.toRadians(180));
    private final Pose intake2EndCloseBlue = new Pose(17.702702702702702,63.594594594594,Math.toRadians(180));
    private final Pose intake3StartCloseBlue = new Pose(55.864864864864856, 39.24324324324324, Math.toRadians(180));
    private final Pose intake3EndCloseBlue = new Pose(20.540540540540533, 39.24324324324324, Math.toRadians(180));
    private final Pose endCloseBlue = new Pose(24.91891891891892,79.97837837837838, Math.toRadians(-135));

    //Blue far
    private final Pose startFarBlue = new Pose(56,8,Math.toRadians(90));
    private final Pose shootFarBlue = new Pose(57.72972972972973,13.513513513513509,Math.toRadians(110));
    private final Pose intake1StartFarBlue = new Pose(55.864864864864856, 37.24324324324324, Math.toRadians(180));
    private final Pose intake1EndFarBlue = new Pose(20.540540540540533, 37.24324324324324, Math.toRadians(180));
    private final Pose intake2StartFarBlue = new Pose(20.91891891891892,44.837837837837846,Math.toRadians(-95));
    private final Pose intake2EndFarBlue = new Pose(18,17.45945945945946,Math.toRadians(-95));
    private final Pose endFarBlue = new Pose(38.108108108108105,26.29729729729731,Math.toRadians(-150));

    //Red close
    private final Pose startCloseRed = new Pose(118.91683145471521,131.24159529402453, Math.toRadians(36.2));
    private final Pose shootCloseRed = new Pose(88.21412875201253,87.34970340213259, Math.toRadians(45));
    private final Pose intake1StartCloseRed = new Pose(91.70270329729729, 87.77027027027026, Math.toRadians(360));
    private final Pose intake1EndCloseRed = new Pose(126.70061523849898, 87.77027027027029, Math.toRadians(360));
    private final Pose intake2StartCloseRed = new Pose(98.91891891891892,63.7027027027027, Math.toRadians(360));
    private final Pose intake2EndCloseRed = new Pose(126.29729729729729,63.5945945945946, Math.toRadians(360));
    //private final Pose intake3StartCloseRed = new Pose(55.864864864864856, 39.24324324324324, Math.toRadians(180)); //TODO
    //private final Pose intake3EndCloseRed = new Pose(20.540540540540533, 39.24324324324324, Math.toRadians(180)); //TODO
    private final Pose endCloseRed = new Pose(112.05405405405405,79.97837837837838, Math.toRadians(250));

    //Red far
    private final Pose startFarRed = new Pose(88,8,Math.toRadians(90));
    private final Pose shootFarRed = new Pose(86.48648648648648,18.05405405405405,Math.toRadians(70));
    private final Pose intake1StartFarRed = new Pose(93.13513513513514, 37.24324324324324, Math.toRadians(0));
    private final Pose intake1EndFarRed = new Pose(126.45945945945947, 37.24324324324324, Math.toRadians(0));
    private final Pose intake2StartFarRed = new Pose(133.75675675675677,38.51351351351348, Math.toRadians(0));
    private final Pose intake2EndFarRed = new Pose(135.43243243243242,8.756756756756735, Math.toRadians(0));
    private final Pose endFarRed = new Pose(38.108108108108105,26.29729729729731,Math.toRadians(275));


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

            LaunchBoard.init(hardwareMap, redAlliance);
        }
        else
        {
            telemetry.addData("", "() - change alliance, [] - change trajectory, X - confirm");

            if (gamepad1.bWasPressed())
            {redAlliance = !redAlliance;}

            if (gamepad1.xWasPressed())
            {closeTrajectory = !closeTrajectory;}

            if (gamepad1.a) {confirmed = true;}

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
