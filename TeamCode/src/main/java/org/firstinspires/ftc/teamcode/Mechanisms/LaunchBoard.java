package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchBoard
{
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo ballStop;


    public void init(HardwareMap hwMap)
    {
        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        ballStop = hwMap.get(Servo.class, "ballStop");
        //Default run mode
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Basically nothing lmao

    //Separate because flywheel needs time to rev
    public void FlywheelMovement(double input) {flywheel.setPower(input);}

    public void BallStopMovement(double input) {ballStop.setPosition(input);}

    public void IntakeMovement(double input) {intake.setPower(input);}

}
