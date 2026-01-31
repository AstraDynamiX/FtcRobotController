package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_DECODE_BLUE")
public class AutonomousBlue extends OpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //Asta e ala de la far
    private Pose startFar = new Pose(56,8,Math.toRadians(90));
    private Pose shootFar = new Pose(57.72972972972973,13.513513513513509,Math.toRadians(110));
    private Pose intake1StartFar = new Pose(55.864864864864856, 37.24324324324324, Math.toRadians(180));
    private Pose intake1EndFar = new Pose(20.540540540540533, 37.24324324324324, Math.toRadians(180));
    private Pose intake2StartFar = new Pose(20.91891891891892,44.837837837837846,Math.toRadians(-95));
    private Pose intake2EndFar = new Pose(18,17.45945945945946,Math.toRadians(-95));
    private Pose endFar = new Pose(38.108108108108105,26.29729729729731,Math.toRadians(-150));

    //Asta e ala de la close
    private Pose startClose = new Pose(25.083168545284792,131.24159529402453,Math.toRadians(143.8));
    private Pose shootClose = new Pose(55.78587124798748,87.34970340213259,Math.toRadians(135));
    private Pose intake1StartClose = new Pose(52.29729670270271, 85.27027027027026, Math.toRadians(180));
    private Pose intake1EndClose = new Pose(19.299384761501017, 85.27027027027029, Math.toRadians(180));
    private Pose intake2StartClose = new Pose(45.08108108108108,59.7027027027027,Math.toRadians(180));
    private Pose intake2EndClose = new Pose(17.702702702702702,59.594594594594,Math.toRadians(180));
    private Pose endClose = new Pose(115.51351351351352,23.108108108108105,Math.toRadians(-135));


    private final Map<String, PathChain> paths = new LinkedHashMap<>();
    Iterator<Map.Entry<String, PathChain>> iterator;
    //Run extra code at the end of each path
    private final List<String> specialIds = new ArrayList<>();
    Iterator<String> idIterator;

    boolean aHeld = false;
    boolean bHeld = false;
    boolean redAlliance = false;
    boolean closeTrajectory = false;
    boolean xHeld = false;
    boolean confirmed = false;

    PathChain path;
    double timerGoal = 0;
    boolean waiting = false;


    @Override
    public void init()
    {
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        opModeTimer.reset();

        OmniBoard.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop()
    {
        if (confirmed)
        {
            telemetry.addData("CONFIRMED", (closeTrajectory) ? "close" : "far");
        }
        else
        {
            telemetry.addData("", "[] - change trajectory, X - confirm");


            if (gamepad1.x && !xHeld)
            {
                xHeld = true;
                closeTrajectory = !closeTrajectory;
            }
            if (!gamepad1.x) {xHeld = false;}

            telemetry.addData("TRAJECTORY", (closeTrajectory) ? "close" : "far");

            if (gamepad1.a)
            {
                confirmed = true;
                LaunchBoard.init(hardwareMap, !redAlliance);
            }
        }
    }

    @Override
    public void start()
    {
        opModeTimer.reset();
        LaunchBoard.start();

        BuildPaths();
        iterator = paths.entrySet().iterator();
        idIterator = specialIds.iterator();
    }

    @Override
    public void loop()
    {
        follower.update();
        StatePathUpdate();
        LaunchBoard.UpdateLaunch(true, 1);
        LaunchBoard.TurretMovement();

        if (opModeTimer.seconds() > 30) {requestOpModeStop();}
    }

    @Override
    public void stop()
    {
        LaunchBoard.stop();
    }


    public void BuildPaths()
    {
        paths.clear();
        specialIds.clear();

        if (closeTrajectory)
        {

            follower.setPose(startClose);

            AddPath(startClose, shootClose, "rev");
            AddPath(shootClose, intake1StartClose, "shoot");
            AddPath(intake1StartClose, intake1EndClose, "intake");
            AddPath(intake1EndClose, shootClose, "rev");
            AddPath(shootClose, intake2StartClose, "shoot");
            AddPath(intake2StartClose, intake2EndClose, "intake");
            AddPath(intake2EndClose, shootClose, "rev");
            AddPath(shootClose, endClose, "shoot");
        }
        else
        {

            follower.setPose(startFar);

            AddPath(startFar, shootFar, "rev");
            AddPath(shootFar, intake1StartFar, "shoot");
            AddPath(intake1StartFar, intake1EndFar, "intake");
            AddPath(intake1EndFar, shootFar, "rev");
            AddPath(shootFar, intake2StartFar, "shoot");
            AddPath(intake2StartFar, intake2EndFar, "intake");
            AddPath(intake2EndFar, shootFar, "revSlow"); //revSlow pt ca miscare e incosistenta
            AddPath(shootFar, endFar, "shoot");
        }
    }



    //Path names = start pose name + end pose name
    private void AddPath(Pose start, Pose end, String specialId)
    {
        String name = PoseName(start) + "_" + PoseName(end);
        paths.put(name, follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build()
        );
        specialIds.add(specialId);
    }

    private void AddPath(Pose start, Pose end) {AddPath(start, end, "");}

    private String PoseName(Pose pose) {return pose.toString();}

    public void StatePathUpdate()
    {
        if (follower.isBusy()) return;

        if (iterator.hasNext() && !waiting)
        {
            Map.Entry<String, PathChain> entry = iterator.next();
            path = entry.getValue();
            String specialId = idIterator.next();

            waiting = true;
            switch (specialId)
            {
                case "intake":
                    follower.setMaxPower(0.5);
                    waiting = false;
                    LaunchBoard.Intake();
                    break;


                case "rev":
                    follower.setMaxPower(1);
                    waiting = false;
                    LaunchBoard.Rev();
                    break;


                case "revSlow":
                    follower.setMaxPower(0.8);
                    waiting = false;
                    LaunchBoard.Rev();
                    break;

                case "shoot":
                    follower.setMaxPower(1);
                    timerGoal = 2000; timer.reset();
                    LaunchBoard.Shoot();
                    break;

                case "":
                default:

                    waiting = false;
                    break;
            }
        }

        if (timer.milliseconds() > timerGoal)
        {
            follower.followPath(path, true);
            waiting = false;
        }
    }
}
