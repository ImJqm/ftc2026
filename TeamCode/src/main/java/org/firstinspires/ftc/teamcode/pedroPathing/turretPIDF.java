package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class turretPIDF {

    private final DcMotorEx motorA;
    private final DcMotorEx motorB;
    double targetVelocity;

    private static final double TICKS_PER_REV = 28.0; // CHANGE if gearbox differs
    private static final double TWO_PI = 2.0 * Math.PI;

    private static final double MAX_COUNTS_PER_SEC =
            (6000.0 / 60.0) * TICKS_PER_REV;

    private static final double MAX_ACCEL = 30000.0; // ticks/sec^2

    private double commandedVelocity = 0.0;

    public turretPIDF(DcMotorEx motorA, DcMotorEx motorB) {
        this.motorA = motorA;
        this.motorB = motorB;

        setupMotor(motorA);
        setupMotor(motorB);

        double kP = 2.0;
        double kI = 0.0;
        double kD = 0.1;
        double kF = 2 * 32767.0 / MAX_COUNTS_PER_SEC;

        motorA.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        motorB.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    private void setupMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update(double targetOmegaRadPerSec, double dt) {

        if (dt <= 0 || dt > 0.1) return;

        targetVelocity =
                targetOmegaRadPerSec * TICKS_PER_REV / TWO_PI;

        targetVelocity = clamp(
                targetVelocity,
                -MAX_COUNTS_PER_SEC,
                 MAX_COUNTS_PER_SEC
        );

        double error = targetVelocity - commandedVelocity;
        double maxDelta = MAX_ACCEL * dt;
        commandedVelocity += clamp(error, -maxDelta, maxDelta);

        motorA.setVelocity(commandedVelocity);
        motorB.setVelocity(commandedVelocity);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getCommandedVelocity() {
        return commandedVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

}
