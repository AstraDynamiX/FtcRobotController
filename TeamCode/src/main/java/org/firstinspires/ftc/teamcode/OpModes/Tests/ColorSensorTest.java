/*package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.OpModes.Global;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp(group = "tests")
public class ColorSensorTest extends OpMode
{
    private static final Logger log = LoggerFactory.getLogger(ColorSensorTest.class);
    LaunchBoard LaunchBoard = new LaunchBoard();

    private boolean ballInRange = false;

    @Override
    public void init()
    {
        LaunchBoard.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        double hue = LaunchBoard.ReadHue(LaunchBoard.ballColor);
        double distance = LaunchBoard.ballDistance.getDistance(DistanceUnit.CM);
        String color = "none";

        LaunchBoard.ballColor.enableLed(true);

        if (distance < 16)
        {
            ballInRange = true;
            if (hue >= 120 + (distance/2) && hue <= 145 + (distance/2)) {color = "green";}
            else {color = "purple";}
        }
        else {ballInRange = false;}

        telemetry.addData("RED:", LaunchBoard.ballColor.red());
        telemetry.addData("BLUE:", LaunchBoard.ballColor.blue());
        telemetry.addData("GREEN:", LaunchBoard.ballColor.green());
        telemetry.addData("HUE:", hue);
        telemetry.addData("BALL IN RANGE:", ballInRange);
        telemetry.addData("COLOR:", color);

    }
}*/