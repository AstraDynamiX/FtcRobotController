package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchBoard
{
    private DcMotor midGecko;
    private CRServo leftGecko;
    private CRServo rightGecko;


    public void init(HardwareMap hwMap)
    {
        //Initializes motors and servos
        leftGecko = hwMap.get(CRServo.class, "leftGecko");
        rightGecko = hwMap.get(CRServo.class, "rightGecko");
        midGecko = hwMap.get(DcMotor.class, "midGecko");
        //Default run mode
        midGecko.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Basically nothing lmao

    //Separate because midGecko needs time to rev
    public void MidGeckoMovement(double input) {midGecko.setPower(input);}

    public void SideGeckoMovement(double input) {leftGecko.setPower(input); rightGecko.setPower(-input);}

}
