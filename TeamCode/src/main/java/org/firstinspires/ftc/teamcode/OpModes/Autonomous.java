package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.CameraBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_DECODE")
public class Autonomous extends OpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();

    ElapsedTime timer = new ElapsedTime();

    private boolean aHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;

    private boolean leftDirection = true;

    private double power = 0.7;

    @Override
    public void init()
    {
        OmniBoard.init(hardwareMap);
        LaunchBoard.init(hardwareMap);
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            leftDirection = !leftDirection;
        }
        if (!gamepad1.a) {aHeld = false;}

        telemetry.addData("DIRECTION", (leftDirection) ? "left" : "right");
    }

    @Override
    public void start()
    {
        timer.reset(); while (timer.milliseconds() < 10000) {}

        power = (leftDirection) ? -0.7 : 0.7;
        OmniBoard.ChassisMovement(0, power, 0, 1);
        timer.reset(); while (timer.milliseconds() < 850) {}

        OmniBoard.ChassisMovement(0, 0, 0, 1);
    }

    @Override
    public void loop() {}
}
