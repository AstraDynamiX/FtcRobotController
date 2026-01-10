package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModes.Global;

public class LaunchBoard {
    private final double FLYWHEEL_KP = 0;
    private final double FLYWHEEL_KI = 0;
    private final double FLYWHEEL_KD = 0;

    private DcMotor intake;
    private DcMotor flywheel;
    private Servo transfer;

    public ColorSensor ballColor;
    public DistanceSensor ballDistance;

    public boolean isShooting = true; //Start with intake
    private boolean littleOuttakeActive = false;

   public IntakeState intakeState = IntakeState.IDLE;
    ElapsedTime intakeTimer = new ElapsedTime();

    ShootingState shootingState = ShootingState.IDLE;
    ElapsedTime transferTimer = new ElapsedTime();

    ElapsedTime flywheelTime = new ElapsedTime();
    private double prevFlywheelSnapshot = 0;
    private double oldFlywheelError = 0;
    private double currentSnapshotRps;
    private double lastSnapshotRps = 0;
    private double flywheelI = 0;

    private double ballAmount = 0;


    public void init(HardwareMap hwMap) {
        Global.indexerSlots = new double[]{0, 0, 0};
        Global.currentPatternSlot = 0;
        //Initializes motors and servos
        intake = hwMap.get(DcMotor.class, "intake");
        flywheel = hwMap.get(DcMotor.class, "flywheel");
        transfer = hwMap.get(Servo.class, "transfer");

        //ballColor = hwMap.get(ColorSensor.class, "ballColor");
        //ballDistance = hwMap.get(DistanceSensor.class, "ballDistance");
        //Default run mode
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SwitchMode();
        intakeTimer.startTime();
    }

    /*public void SwitchMode()
    {
        isShooting = !isShooting;
        if (isShooting)
        {
            ballAmount = 3;
            FlywheelMovement(0.8);
            if (intakeState != IntakeState.FULL) {intakeState = IntakeState.IDLE;}
            shootingState = ShootingState.TRANSFER_UP;
        }
        else
        {
            FlywheelMovement(0);
            if (intakeState != IntakeState.FULL) {intakeState = IntakeState.EATING;}
            shootingState = ShootingState.IDLE;
        }
    }*/
    public void IntakeMovement(double input) {intake.setPower(input);}

    public void LittleOuttake(double input) {
            intakeTimer.reset();
            intakeTimer.startTime();
            littleOuttakeActive = true;
            intake.setPower(input);

    }

    public void updateLittleOuttake() {
        if (littleOuttakeActive && intakeTimer.milliseconds() >= 7) {
            intake.setPower(0);
            littleOuttakeActive = false;
        }
    }

    public void FlywheelMovement(double input)
    {

        /*double flywheelDeltaTime = flywheelTime.seconds() - prevFlywheelSnapshot;
        double targetRps = input * (flywheel.getMotorType().getMaxRPM() / 60);
        currentSnapshotRps = (flywheel.getCurrentPosition() / flywheel.getMotorType().getTicksPerRev()) / flywheelDeltaTime) - lastSnapshotRps;
        double error = targetRps - currentSnapshotRps;

        double P = FLYWHEEL_KP * error;
        double D = FLYWHEEL_KD * (error - oldFlywheelError);
        flywheelI += FLYWHEEL_KI * error;

        flywheel.setPower(P + flywheelI + D);
        lastSnapshotRps = currentSnapshotRps;
        oldFlywheelError = error;*/
        flywheel.setPower(input);
    }

    public double GetCurrentSnapshotRps() {return currentSnapshotRps;}

    public double ReadHue(ColorSensor sensor)
    {
        //Normalize RGB values to range [0, 1]
        double hue;
        float red = sensor.red() / 550f;
        float green = sensor.green() / 600f; //Sensor seems to have a bias towards green
        float blue = sensor.blue() / 550f;

        // Find the maximum and minimum values among RGB
        float maxColor = Math.max(red, Math.max(green, blue));
        float minColor = Math.min(red, Math.min(green, blue));
        float deltaColor = maxColor - minColor;

        // Calculate the hue
        if (deltaColor == 0) {hue = 0;}
        else if (maxColor == red) {hue = ((green - blue) / deltaColor) % 6;}
        else if (maxColor == green) {hue = ((blue - red) / deltaColor) + 2;}
        else {hue = ((red - green) / deltaColor) + 4;}

        hue *= 60;
        if (hue < 0) {hue += 360;}

        return hue;
    }

    // --- Intake state machine ---
    enum IntakeState
    {
        IDLE,
        EATING,
        FULL
    }

    public void UpdateIntake()
    {
        switch (intakeState)
        {
            case EATING:

                //intake.setPower(-0.6);

                /*double distance = ballDistance.getDistance(DistanceUnit.CM);
                double hue = ReadHue(ballColor);

                if (distance < 5)
                {
                    //distance = 5 => closest to color sensor => highest hues
                    if (hue >= 125 + (distance / 3) && hue <= 145 + (distance / 3))
                    {Global.indexerSlots[Global.currentSlot] = 2;} //gren
                    else {Global.indexerSlots[Global.currentSlot] = 1;} //puple
                }*/

                break;

            //case FULL:
            case IDLE:
            default:

                //intake.setPower(0);
                break;
        }
    }

    // --- Shooting state machine ---
    enum ShootingState
    {
        IDLE,
        TRANSFER_UP,
        TRANSFER_DOWN
    }

    public void StartShooting()
    {
        //ballAmount = 3;
        shootingState = ShootingState.TRANSFER_UP;
    }

    public void StopShooting() {shootingState = ShootingState.IDLE;}


    public void UpdateShooting()
    {
        switch (shootingState)
        {
            case TRANSFER_UP:

                if (transferTimer.milliseconds() <= 500) {break;}

                //if (ballDistance.getDistance(DistanceUnit.CM) < 5)
                {
                    transfer.setPosition(0.025);
                    transferTimer.reset();
                    shootingState = ShootingState.TRANSFER_DOWN;
                }
                break;

            case TRANSFER_DOWN:

                if (transferTimer.milliseconds() <= 350) {break;}

                transfer.setPosition(0.25);
                transferTimer.reset();

                /*if (ballAmount > 0)
                {
                    ballAmount--;
                    shootingState = ShootingState.TRANSFER_UP;
                }
                else*/
                {shootingState = ShootingState.IDLE;}
                break;

            case IDLE:
            default:

                break;
        }
    }

}