package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Turret Tuner")
public class turretTuner extends OpMode{
    
    DcMotorEx motorChain;
    DcMotorEx motorBevel;
    double lastTime;

    Servo outTakeServo;

    double amount;

    turretPIDF tuner;

    @Override
    public void init() {
        // Initialize the turretPIDF
        motorChain = hardwareMap.get(DcMotorEx.class, "topLauncher");
        motorChain.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBevel = hardwareMap.get(DcMotorEx.class, "sideLauncher");
        tuner = new turretPIDF(motorChain, motorBevel);
        lastTime = getRuntime();
        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        outTakeServo.setPosition(0.33);
        amount = 0.0;
    }

    @Override
    public void loop() {

        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // Prevent bad dt values
        if (dt <= 0 || dt > 0.1) return;

        // Joystick input (-1 to 1)
        double joystick = gamepad1.right_stick_x;
        telemetry.addData("Right Joy Stik:", joystick);
        telemetry.addData("dt:", dt);
        telemetry.addData("Command Velocity:", tuner.getCommandedVelocity());
        telemetry.addData("Target Velocity:", tuner.getTargetVelocity());
        telemetry.addData("Amount:", amount);
        if (gamepad1.rightBumperWasPressed()) {
            amount+=1.0;
        }
        if (gamepad1.leftBumperWasPressed()) {
            amount-=1.0;
        }

        outTakeServo.setPosition(gamepad1.dpad_up ? 0.1 : 0.33);

        // Max turret angular velocity (rad/s)
        double maxOmega = 100 * Math.PI*2.0;

        // Target angular velocity
        double targetOmega = amount/100 * joystick * maxOmega;

        // Update velocity controller
        tuner.update(targetOmega, dt);
    }

}
