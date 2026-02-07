package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {

    private Servo outTakeServo;

    private DcMotorEx topMotor;

    private DcMotorEx sideMotor;

    private  DcMotor midtakeMotor;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum wheelState {
        IDLE,
        SPINUP,
        LAUNCH,
        RESET_GATE
    }


    private wheelState flywheelState;

    private int targetRPM = 2600;

    private int targetRPS = targetRPM/60;

    private int minRPM = 2400;

    private int minRPS = minRPM/60;

    private int ticks = 28;

    private int shotsRemaining = 0;

    private double servoRest = 0.33;

    private double servoActuate = 0.10;

    private double flywheelShootPower = 0.55;

    //Time:
    private double flywheelAccelTime = 1.4; //seconds

    private double servoActuateTime = 0.9;

    public void init(HardwareMap hardwareMap) {
        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        topMotor = hardwareMap.get(DcMotorEx.class, "topLauncher");
        sideMotor = hardwareMap.get(DcMotorEx.class, "sideLauncher");
        midtakeMotor = hardwareMap.get(DcMotor.class, "vmotor2");

        sideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = wheelState.IDLE;
        outTakeServo.setPosition(servoRest);
    }

    public void update() {

        switch (flywheelState) {
            case IDLE:
                midtakeMotor.setPower(0.0);
                if (shotsRemaining > 0) {

                    outTakeServo.setPosition(servoRest);



                    topMotor.setPower(flywheelShootPower);
                    sideMotor.setPower(flywheelShootPower);

                    //topMotor.setVelocity(ticks*targetRPS);
                    //sideMotor.setVelocity(ticks*targetRPS);


                    //Here we want to make the motor aim for its target velocity





                    stateTimer.reset();

                    flywheelState = wheelState.SPINUP;
                }
                break;
            case SPINUP:
                //set velocity again if want
                //topMotor.setVelocity(ticks*targetRPS);
                //sideMotor.setVelocity(ticks*targetRPS);



                if (/*flywheel velocity > min vel*/ /*(topMotor.getVelocity() > (ticks*minRPS) && sideMotor.getVelocity() > (ticks*minRPS))   ||*/stateTimer.seconds() > flywheelAccelTime) {
                    outTakeServo.setPosition(servoActuate);
                    shotsRemaining--;
                    stateTimer.reset();
                    flywheelState = wheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > servoActuateTime) {
                    if (shotsRemaining > 0) {
                        if (shotsRemaining == 1) {
                            midtakeMotor.setPower(1.0);
                        }

                        outTakeServo.setPosition(servoRest);
                        stateTimer.reset();
                        flywheelState = wheelState.SPINUP;

                    } else {
                       // topMotor.setVelocity(0.0);
                        // sideMotor.setVelocity(0.0);
                        topMotor.setPower(0.0);
                        sideMotor.setPower(0.0);
                        outTakeServo.setPosition(servoRest);
                        stateTimer.reset();
                        ;
                        flywheelState = wheelState.IDLE;
                    }
                }
                break;
        }
    }

    public void fireShots(int shots) {
        if (flywheelState == wheelState.IDLE) {
            shotsRemaining = shots;
        }
    }

    public boolean isBusy() {
        return flywheelState != wheelState.IDLE;
    }

    public String getState() {
        return flywheelState.toString();
    }

    public Double getVelocity(int index) {
        switch (index) {
            case 0: //Top Motor
              //  return (topMotor.getVelocity()*60/ticks);
                return (sideMotor.getPower());

            case 1: //Side Motor
//                return (sideMotor.getVelocity()*60/ticks);
                return (sideMotor.getPower());

            default:
                return 0.0;

        }
    }


}
