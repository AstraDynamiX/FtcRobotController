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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_DECODE_RED")
public class AutonomousRed extends OpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //Asta e ala de la far
    private Pose startFar = new Pose(88,8,Math.toRadians(90));
    private Pose shootFar = new Pose(86.48648648648648,18.05405405405405,Math.toRadians(70));
    private Pose intake1StartFar = new Pose(93.13513513513514, 37.24324324324324, Math.toRadians(0));
    private Pose intake1EndFar = new Pose(126.45945945945947, 37.24324324324324, Math.toRadians(0));
    private Pose intake2StartFar = new Pose(133.75675675675677,38.51351351351348, Math.toRadians(0));
    private Pose intake2EndFar = new Pose(135.43243243243242,8.756756756756735, Math.toRadians(0));
    private Pose endFar = new Pose(38.108108108108105,26.29729729729731,Math.toRadians(275));

    //Asta e ala de la close
    private Pose startClose = new Pose(118.91683145471521,131.24159529402453,Math.toRadians(36.19999999999999));
    private Pose shootClose = new Pose(88.21412875201253,87.34970340213259,Math.toRadians(45));
    private Pose intake1StartClose = new Pose(91.70270329729729, 89.77027027027026, Math.toRadians(360));
    private Pose intake1EndClose = new Pose(126.70061523849898, 89.77027027027029, Math.toRadians(360));
    private Pose intake2StartClose = new Pose(98.91891891891892,65.7027027027027,Math.toRadians(360));
    private Pose intake2EndClose = new Pose(126.29729729729729,65.5945945945946,Math.toRadians(360));
    private Pose endClose = new Pose(112.05405405405405,67.72972972972973,Math.toRadians(250));


    private final Map<String, PathChain> paths = new LinkedHashMap<>();
    Iterator<Map.Entry<String, PathChain>> iterator;
    //Run extra code at the end of each path
    private final List<String> specialIds = new ArrayList<>();
    Iterator<String> idIterator;

    boolean aHeld = false;
    boolean bHeld = false;
    boolean redAlliance = true;
    boolean closeTrajectory = false;
    boolean xHeld = false;
    boolean confirmed = false;
    boolean camAdjustment = true;

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
        LaunchBoard.init(hardwareMap,true);

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
        if(camAdjustment) {
            LaunchBoard.UpdateLaunch(true, 1);
            LaunchBoard.TurretLockPosition(0);
        }
        else
        {
            LaunchBoard.TurretLockPosition(0);
            LaunchBoard.UpdateLaunch(false,1);
        }

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
            AddPath(shootClose, intake2StartClose, "shootNoCam");
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
                    camAdjustment = false;
                    waiting = false;
                    LaunchBoard.Intake();
                    break;


                case "rev":
                    follower.setMaxPower(1);
                    camAdjustment = true;
                    waiting = false;
                    LaunchBoard.Rev();
                    break;




                case "revSlow":
                    follower.setMaxPower(0.8);
                    camAdjustment = true;
                    waiting = false;
                    LaunchBoard.Rev();
                    break;

                case "shoot":
                    follower.setMaxPower(1);
                    camAdjustment = true;
                    timerGoal = 2000; timer.reset();
                    LaunchBoard.Shoot();
                    break;

                case "shootNoCam":
                    follower.setMaxPower(1);
                    camAdjustment = false;
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
