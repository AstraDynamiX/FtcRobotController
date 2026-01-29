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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_DECODE")
public class Autonomous extends OpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();

    private Follower follower;

    ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime opModeTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    private boolean aHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private double location = 0;

    public enum PathState
    {
        // START POSITION_END POSITION
        // DRIVE -> MOVEMENT STATE
        // SHOOT -> ATTEMPT TO SHOOT ARTEFACT
        DRIVE_STARTPOSE_SHOOT_POSE,
        SHOOT_PRELOAD,
        INTAKE1_ROTATE,
        INTAKE1,
        SHOOT_POSE1,
        SHOOT1,
        INTAKE2,
        INTAKE2_END,
        SHOOT_POSE2,
        SHOOT2,
        PARK

    }

    PathState pathState;

    private final Pose startpose = new Pose(25.083168545284792,131.24159529402453,Math.toRadians(143.8));
    private final Pose shootpose = new Pose(55.78587124798748,87.34970340213259,Math.toRadians(135));
    private final Pose intakepose1rotate = new Pose(52.29729670270271, 85.27027027027026, Math.toRadians(-180));
    private final Pose intakepose1 = new Pose(19.299384761501017, 85.27027027027029, Math.toRadians(-180));
    private final Pose intakepose2 = new Pose(55.78587124798748,87.34970340213259,Math.toRadians(-180));
    private final Pose intakepose2end = new Pose(45.08108108108108,59.7027027027027,Math.toRadians(-180));
    private final Pose exitpose = new Pose(115.51351351351352,23.108108108108105,Math.toRadians(-135));


    private PathChain driverStartPosShootPos,
            driverShootPosIntake1RotatePos,
            driverIntake1RotatePosIntake1Pos,
            driverIntake1PosShootPos,
            driverShootPosIntake2Pos,
            driverIntake2PosIntake2FinalPos,
            driverIntake2FinalPosShootPos,
            driverShootPosExitPos ;

    public void buildPaths()
    {
        //coordonates for starting pose -> ending pose
        driverStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startpose, shootpose))
                .setLinearHeadingInterpolation(startpose.getHeading(), shootpose.getHeading())
                .build();
        driverShootPosIntake1RotatePos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose, intakepose1rotate))
                .setLinearHeadingInterpolation(shootpose.getHeading(), intakepose1rotate.getHeading())
                .build();
        driverIntake1RotatePosIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(intakepose1rotate, intakepose1))
                .setLinearHeadingInterpolation(intakepose1rotate.getHeading(), intakepose1.getHeading())
                .build();
        driverIntake1PosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakepose1, shootpose))
                .setLinearHeadingInterpolation(intakepose1.getHeading(), shootpose.getHeading())
                .build();
        driverShootPosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose, intakepose2))
                .setLinearHeadingInterpolation(shootpose.getHeading(), intakepose2.getHeading())
                .build();
        driverIntake2PosIntake2FinalPos = follower.pathBuilder()
                .addPath(new BezierLine(intakepose2, intakepose2end))
                .setLinearHeadingInterpolation(intakepose2.getHeading(), intakepose2end.getHeading())
                .build();
        driverIntake2FinalPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakepose2end, shootpose))
                .setLinearHeadingInterpolation(intakepose2end.getHeading(), shootpose.getHeading())
                .build();
        driverShootPosExitPos= follower.pathBuilder()
                .addPath(new BezierLine(shootpose, exitpose))
                .setLinearHeadingInterpolation(shootpose.getHeading(), exitpose.getHeading())
                .build();

    }

    public void statePathUpdate()
    {
        switch (pathState) {
            case DRIVE_STARTPOSE_SHOOT_POSE:
                follower.followPath(driverStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                //reset the timer + make new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.milliseconds() > 5000) {

                    /*+SHOOTING*/
                    setPathState(PathState.INTAKE1_ROTATE);
                }
                break;
            case INTAKE1_ROTATE:
                if (!follower.isBusy()) {
                    follower.followPath(driverShootPosIntake1RotatePos, true);
                    setPathState(PathState.INTAKE1);
                }
                break;

            case INTAKE1:
                if (!follower.isBusy()) {
                    follower.followPath(driverIntake1RotatePosIntake1Pos, true);
                    setPathState(PathState.SHOOT_POSE1);
                }
                break;
            case SHOOT_POSE1:
                if (!follower.isBusy()) {
                    follower.followPath(driverIntake1PosShootPos, true);
                    setPathState(PathState.SHOOT1);

                }
                break;
            case SHOOT1:
                if (!follower.isBusy() && pathTimer.milliseconds() > 5000) {

                    /*+SHOOTING*/
                    setPathState(PathState.INTAKE2);
                }
                break;
            case INTAKE2:
                if (!follower.isBusy()) {
                    follower.followPath(driverShootPosIntake2Pos, true);
                    setPathState(PathState.INTAKE2_END);

                }
                break;
            case INTAKE2_END:
                if (!follower.isBusy()) {
                    follower.followPath(driverIntake2PosIntake2FinalPos, true);
                    setPathState(PathState.SHOOT_POSE2);

            }
            break;
            case SHOOT_POSE2:
                if (!follower.isBusy()) {
                    follower.followPath(driverIntake2FinalPosShootPos, true);
                    setPathState(PathState.SHOOT2);

                }
                break;
            case SHOOT2:
                if (!follower.isBusy() && pathTimer.milliseconds() > 2000) {
                    /*+SHOOTING*/
                    setPathState(PathState.PARK);
                }


            case PARK:
                if (!follower.isBusy()) {
                    follower.followPath(driverShootPosExitPos, true);

                }
                break;
        }




    }

    public void setPathState (PathState newstate)
    {
        pathState = newstate;
        pathTimer.reset();

    }

    @Override
    public void init()
    {
        pathState = PathState.DRIVE_STARTPOSE_SHOOT_POSE;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        opModeTimer.reset();

        OmniBoard.init(hardwareMap);
        LaunchBoard.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startpose);
    }


//    @Override
//    public void init_loop()
//    {
//        if (gamepad1.dpad_up && upHeld)
//        {
//            upHeld = true;
//
//        }
//        if (!gamepad1.dpad_up) {upHeld = false;}
//
//        if (gamepad1.dpad_down && downHeld)
//        {
//            downHeld = true;
//
//        }
//        if (!gamepad1.dpad_down) {downHeld = false;}
//    }

    @Override
    public void start(){
        opModeTimer.reset();
        setPathState(pathState);

    }


    @Override
    public void loop()
    {
        follower.update();
        statePathUpdate();
    }
}
