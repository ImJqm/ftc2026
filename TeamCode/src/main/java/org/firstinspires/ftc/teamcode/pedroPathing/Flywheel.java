package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {

    private Servo outTakeServo;

    private DcMotor topMotor;

    private DcMotor sideMotor;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum wheelState {
        IDLE,
        SPINUP,
        LAUNCH,
        RESET_GATE
    }

    private wheelState flywheelState;

    private int shotsRemaining = 0;

    private double servoRest = 0.43;

    private double servoActuate = 0.20;

    private double flywheelShootPower = 0.55;

    //Time:
    private double flywheelAccelTime = 1.3; //seconds

    private double servoActuateTime = 1.0;
    public void init(HardwareMap hardwareMap) {
        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        topMotor = hardwareMap.get(DcMotor.class, "topLauncher");
        sideMotor = hardwareMap.get(DcMotor.class, "sideLauncher");

        sideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelState = wheelState.IDLE;
        outTakeServo.setPosition(servoRest);
    }

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {

                    outTakeServo.setPosition(servoRest);

                    topMotor.setPower(flywheelShootPower);
                    //sideMotor.setPower(flywheelShootPower);
                    topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    stateTimer.reset();

                    flywheelState = wheelState.SPINUP;
                }
                break;
            case SPINUP:
                //set velocity again if want
                if (/*flywheel velocity > min vel*/ stateTimer.seconds() > flywheelAccelTime) {
                    shotsRemaining--;
                    stateTimer.reset();
                    flywheelState = wheelState.LAUNCH;
                }
                break;
            case LAUNCH:


        }
    }

}
