package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Tuner")
public class turretTuner extends OpMode {

    DcMotorEx motorChain;
    DcMotorEx motorBevel;
    Servo outTakeServo;

    turretPIDF tuner;

    double amount = 1.0;

    @Override
    public void init() {

        motorChain = hardwareMap.get(DcMotorEx.class, "topLauncher");
        motorBevel = hardwareMap.get(DcMotorEx.class, "sideLauncher");

        motorChain.setDirection(DcMotorSimple.Direction.REVERSE);

        tuner = new turretPIDF(motorChain, motorBevel);

        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        outTakeServo.setPosition(0.33);
    }

    @Override
    public void loop() {

        double joystick = gamepad1.right_stick_x;

        // ===== Scale Adjustment =====
        if (gamepad1.rightBumperWasPressed()) amount += 0.01;
        if (gamepad1.leftBumperWasPressed())  amount -= 0.01;

        // ===== Live PID Tuning =====
        if (gamepad1.yWasPressed()) tuner.changeP(0.1);
        if (gamepad1.aWasPressed()) tuner.changeP(-0.1);

        if (gamepad1.bWasPressed()) tuner.changeD(0.01);
        if (gamepad1.xWasPressed()) tuner.changeD(-0.01);

        if (gamepad1.dpadUpWasPressed()) tuner.changeF(1.0);
        if (gamepad1.dpadDownWasPressed()) tuner.changeF(-1.0);

        tuner.applyPIDF();

        // ===== Servo Test =====
        outTakeServo.setPosition(gamepad1.dpad_left ? 0.1 : 0.33);

        // ===== Target Velocity =====
        double maxOmega = 2.0 * Math.PI * 100; // adjust to real max
        double targetOmega = amount * joystick * maxOmega;

        tuner.update(targetOmega);

        // ===== Telemetry =====
        telemetry.addData("Target Omega", targetOmega);
        telemetry.addData("Measured Velocity", tuner.getMeasuredVelocity());
        telemetry.addData("P", tuner.getP());
        telemetry.addData("D", tuner.getD());
        telemetry.addData("F", tuner.getF());
        telemetry.addData("Scale", amount);
        telemetry.update();
    }
}