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
    CameraBoard CamBoard = new CameraBoard();

    ElapsedTime timer = new ElapsedTime();

    private boolean aHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;

    private boolean blueAlliance = true;
    private double batteryVoltage = 12.7;

    private double power = 0.7;

    @Override
    public void init()
    {
        OmniBoard.init(hardwareMap);
        LaunchBoard.init(hardwareMap);
        CamBoard.init(hardwareMap);
    }

    @Override
    public void init_loop()
    {
        if (gamepad1.a && !aHeld)
        {
            aHeld = true;
            blueAlliance = !blueAlliance;
        }
        if (!gamepad1.a) {aHeld = false;}

        if (gamepad1.dpad_up && !upHeld)
        {
            upHeld = true;
            batteryVoltage += 0.1;
        }
        if (!gamepad1.dpad_up) {upHeld = false;}

        if (gamepad1.dpad_down && !downHeld)
        {
            downHeld = true;
            batteryVoltage -= 0.1;
        }
        if (!gamepad1.dpad_down) {downHeld = false;}

        telemetry.addData("BATTERY VOLTAGE", batteryVoltage);
        telemetry.addData("ALLIANCE", (blueAlliance) ? "blue" : "red");
    }

    @Override
    public void start()
    {
        timer.reset(); while (timer.milliseconds() < 10000) {}

        power = (blueAlliance) ? -0.6 : 0.6;
        OmniBoard.ChassisMovement(0, power, 0, 1);
        timer.reset(); while (timer.milliseconds() < 1000) {}

        OmniBoard.ChassisMovement(0, 0, 0, 1);
    }

    @Override
    public void loop() {}
}
