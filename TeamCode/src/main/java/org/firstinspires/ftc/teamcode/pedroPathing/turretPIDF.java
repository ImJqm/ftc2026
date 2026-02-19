package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;


public class turretPIDF {

    private final DcMotorEx motorChain;
    private final DcMotorEx motorBevel;

    private static final double TICKS_PER_REV = 28.0;     // PPR at output
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double MAX_COUNTS_PER_SEC = (6000.0 / 60.0) * TICKS_PER_REV;

    // PIDF gains for inner velocity loop
    private static  double kP = 4.25;
    private static  double kI = 0.0;
    private static  double kD = 0.1;
    private static final double kF = 32767.0 / MAX_COUNTS_PER_SEC;

    // Boost factor for outer loop
    private static final double kBoost = 0.50; // 0.1-0.5 recommended; increases acceleration

    private double lastTargetVelocity = 0.0;

    public turretPIDF(DcMotorEx motorChain, DcMotorEx motorBevel) {
        this.motorChain = motorChain;
        this.motorBevel = motorBevel;

        motorChain.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBevel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorChain.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBevel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorChain.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        motorBevel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    /**
     * Update turret velocity with boosted target for fast acceleration
     * @param targetOmegaRadPerSec true desired target angular velocity (rad/s)
     */
    public void update(double targetOmegaRadPerSec) {

        motorChain.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        motorBevel.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Convert rad/s → encoder ticks/sec
        double targetVelocity = targetOmegaRadPerSec * TICKS_PER_REV / TWO_PI;

        // Current measured velocity
        double measuredVelocity = (motorChain.getVelocity() + motorBevel.getVelocity()) / 2.0;

        // ===== Outer loop boost =====
        double boostedVelocity = targetVelocity + kBoost * (targetVelocity - measuredVelocity);

        // Clamp to motor limits
        boostedVelocity = Range.clip(boostedVelocity, -MAX_COUNTS_PER_SEC, MAX_COUNTS_PER_SEC);

        // ===== Inner loop =====
        motorChain.setVelocity(boostedVelocity);
        motorBevel.setVelocity(boostedVelocity);

        lastTargetVelocity = targetVelocity;
    }

    // Telemetry helpers
    public double getMeasuredVelocity() {
        return (motorChain.getVelocity() + motorBevel.getVelocity()) / 2.0;
    }

    public double getTargetVelocity() {
        return lastTargetVelocity;
    }

    public void changeP(double delta) {
        kP +=delta;
    }

    public double getP() {
        return kP;
    }

    public void changeI(double delta) {
        kI +=delta;
    }

    public double getI() {
        return kI;
    }

    public void changeD(double delta) {
        kD +=delta;
    }

    public double getD() {
        return kD;
    }


}