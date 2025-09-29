package OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.LaunchBoard;
import org.firstinspires.ftc.teamcode.Mechanisms.OmnimovementBoard;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous_blueBasket")
public class AutonomousBlueBasket extends LinearOpMode
{
    OmnimovementBoard OmniBoard = new OmnimovementBoard();
    LaunchBoard LaunchBoard = new LaunchBoard();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        //Init
        OmniBoard.init(hardwareMap); OmniBoard.initAuto(this);
        LaunchBoard.init(hardwareMap);
        telemetry.addData("IMU:", OmniBoard.getIntegratedHeading());
        waitForStart();

        //Moving
        LaunchBoard.MidGeckoMovement(0.85);
        OmniBoard.IMUFollow(375, -60, 0);
        LaunchAll();

        OmniBoard.IMUFollow(600, 0, -60);
    }

    private void LaunchAll()
    {
        for (int i = 0; i < 3; i++)
        {
            LaunchBoard.SideGeckoMovement(0.85);
            timer.reset(); while (timer.milliseconds() <= 650) {}
            LaunchBoard.SideGeckoMovement(0);
            timer.reset(); while (timer.milliseconds() <= 600) {}
        }
    }
}
