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

    double amount;
    turretPIDF tuner;

    @Override
    public void init() {

        motorChain = hardwareMap.get(DcMotorEx.class, "topLauncher");
        motorChain.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBevel = hardwareMap.get(DcMotorEx.class, "sideLauncher");

        tuner = new turretPIDF(motorChain, motorBevel);
        motorChain.setDirection(DcMotorSimple.Direction.REVERSE);

        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        outTakeServo.setPosition(0.33);

        amount = 1.0;  // start at full scaling
    }

    @Override
    public void loop() {

        double joystick = gamepad1.right_stick_x;

        if (gamepad1.rightBumperWasPressed()) {
            amount += 0.01;
        }
        if (gamepad1.leftBumperWasPressed()) {
            amount -= 0.01;
        }

        outTakeServo.setPosition(gamepad1.dpad_up ? 0.1 : 0.33);

        double maxOmega = 100 * Math.PI * 2.0;
        double targetOmega = amount * joystick * maxOmega;

        tuner.update(targetOmega);

        telemetry.addData("Joystick", joystick);
        telemetry.addData("Omega Target", targetOmega);
        telemetry.addData("Measured Velocity", tuner.getMeasuredVelocity());
        telemetry.addData("Amount Scale", amount);
        telemetry.update();
    }
}
