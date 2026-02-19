package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class turretPIDF {

    private final DcMotorEx motorChain;
    private final DcMotorEx motorBevel;

    private static final double TICKS_PER_REV = 28.0;
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double MAX_COUNTS_PER_SEC = (6000.0 / 60.0) * TICKS_PER_REV;

    // ===== PIDF Gains =====
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    // Start with theoretical kF — tune this first
    private double kF = 32767.0 / MAX_COUNTS_PER_SEC;

    private double lastTargetVelocity = 0.0;

    public turretPIDF(DcMotorEx motorChain, DcMotorEx motorBevel) {
        this.motorChain = motorChain;
        this.motorBevel = motorBevel;

        motorChain.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBevel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorChain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBevel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        applyPIDF();
    }

    // ===== Apply Gains to Hardware =====
    public void applyPIDF() {
        motorChain.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        motorBevel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // ===== Update Velocity =====
    public void update(double targetOmegaRadPerSec) {

        double targetVelocity = targetOmegaRadPerSec * TICKS_PER_REV / TWO_PI;

        targetVelocity = Range.clip(targetVelocity,
                -MAX_COUNTS_PER_SEC,
                MAX_COUNTS_PER_SEC);

        motorChain.setVelocity(targetVelocity);
        motorBevel.setVelocity(targetVelocity);

        lastTargetVelocity = targetVelocity;
    }

    // ===== Telemetry Helpers =====
    public double getMeasuredVelocity() {
        return (motorChain.getVelocity() + motorBevel.getVelocity()) / 2.0;
    }

    public double getTargetVelocity() {
        return lastTargetVelocity;
    }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }

    public void changeP(double delta) { kP += delta; }
    public void changeI(double delta) { kI += delta; }
    public void changeD(double delta) { kD += delta; }
    public void changeF(double delta) { kF += delta; }
}