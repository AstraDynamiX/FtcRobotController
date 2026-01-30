package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.CameraBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_DECODE")
public class Autonomous extends OpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    private final Pose start = new Pose(25.083168545284792,131.24159529402453,Math.toRadians(143.8));
    private final Pose shoot = new Pose(55.78587124798748,87.34970340213259,Math.toRadians(135));
    private final Pose intake1Start = new Pose(52.29729670270271, 85.27027027027026, Math.toRadians(-180));
    private final Pose intake1End = new Pose(19.299384761501017, 85.27027027027029, Math.toRadians(-180));
    private final Pose intake2Start = new Pose(45.08108108108108,59.7027027027027,Math.toRadians(-180));
    private final Pose intake2End = new Pose(17.702702702702702,59.594594594594,Math.toRadians(-180));
    private final Pose end = new Pose(115.51351351351352,23.108108108108105,Math.toRadians(-135));

    private final Map<String, PathChain> paths = new LinkedHashMap<>();
    Iterator<Map.Entry<String, PathChain>> iterator;
    //Run extra code at the end of each path
    private List<String> specialIds;
    Iterator<String> idIterator;

    boolean aHeld = false;
    boolean redAlliance = false;

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
        BuildPaths();
        follower.setPose(start);

        iterator = paths.entrySet().iterator();
        idIterator = specialIds.iterator();
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            redAlliance = !redAlliance;
        }
        if(!gamepad1.a) {aHeld = false;}

        telemetry.addData("ALLIANCE", (redAlliance) ? "red" : "blue");
    }


    @Override
    public void start()
    {
        opModeTimer.reset();
        LaunchBoard.init(hardwareMap, redAlliance);
    }

    @Override
    public void loop()
    {
        follower.update();
        StatePathUpdate();

        if (opModeTimer.seconds() > 30) {requestOpModeStop();}
    }


    public void BuildPaths()
    {
        AddPath(start, shoot);
        AddPath(shoot, intake1Start, "shoot");
        AddPath(intake1Start, intake1End, "intake");
        AddPath(intake1End, shoot, "rev");
        AddPath(shoot, intake2Start, "shoot");
        AddPath(intake2Start, intake2End, "intake");
        AddPath(intake2Start, shoot, "rev");
        AddPath(shoot, end, "shoot");
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

                    waiting = false;
                    LaunchBoard.Intake();
                    break;

                case "rev":

                    waiting = false;
                    LaunchBoard.Rev();
                    break;

                case "shoot":

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
