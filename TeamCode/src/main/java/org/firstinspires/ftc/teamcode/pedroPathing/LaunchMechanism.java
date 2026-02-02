package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Toggle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Launch Mechanism")
public class LaunchMechanism extends OpMode {

    Servo outTakeServo;

    IMU imu;
    DcMotor motorFrL;
    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor intakeMotor;
    DcMotor midtakeMotor;
    DcMotor topMotor;
    DcMotor sideMotor;
    Toggle aToggle = new Toggle();
    Toggle bToggle = new Toggle();

    Toggle yToggle = new Toggle();

    Toggle rbToggle = new Toggle();

    Toggle aToggle2 = new Toggle();

    Toggle upToggle = new Toggle();
    Toggle bToggle2 = new Toggle();
    boolean intakeMode;

    boolean slowmode = false;
    int count = 0;

    int countercount;
    boolean counterActuating;
    int servoCount = 0;

    double outtakePower = 0.55;

    boolean launching1 = false;
    boolean launching2 = false;
    boolean actuating = false;

    boolean reverse = true;


    @Override
    public void init() {

        motorFrL = hardwareMap.get(DcMotor.class,"lf");
        motorFrR = hardwareMap.get(DcMotor.class,"rf");
        motorBL = hardwareMap.get(DcMotor.class,"lr");
        motorBR = hardwareMap.get(DcMotor.class,"rr ");

        intakeMotor = hardwareMap.get(DcMotor.class, "vmotor1");
        midtakeMotor = hardwareMap.get(DcMotor.class, "vmotor2");
        topMotor = hardwareMap.get(DcMotor.class, "topLauncher");
        sideMotor = hardwareMap.get(DcMotor.class, "sideLauncher");
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrL.setDirection(DcMotorSimple.Direction.REVERSE);
        midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        outTakeServo.setPosition(0.43);


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD ));
        imu.initialize(parameters);

    }


    @Override
    public void loop() {


        slowmode = aToggle2.update(gamepad2.a);

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        if (gamepad2.options) {
            imu.resetYaw();
        }
        telemetry.addData("Gamepad y: ", gamepad1.y);
        telemetry.addData("Launching: ", launching2 );
        telemetry.addData("Direction of Midtake: ", midtakeMotor.getDirection());
        telemetry.addData("Outtake Power", outtakePower);


        if (gamepad1.right_bumper && !counterActuating) {
            counterActuating = true;
            countercount = 0;
            outtakePower+=0.05;
        } else if (countercount>30) {
            counterActuating = false;
        }

        if (gamepad1.left_bumper && !counterActuating) {
            counterActuating = true;
            countercount = 0;
            outtakePower-=0.05;
        } else if (countercount>30) {
            counterActuating = false;
        }



        if (gamepad1.dpad_up && !launching2 && !launching1) {
            launching1 = true;
            midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            midtakeMotor.setPower(0.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
            count = 0;
        } else if (count < 50 && launching1) {
            midtakeMotor.setPower(0.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
        } else if (count < 80 && launching1) {
            midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            midtakeMotor.setPower(1.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(1.0);
        }else if (count >= 40){
            launching1 = false;
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
            midtakeMotor.setPower(0.0);
        }

        reverse = bToggle2.update(gamepad2.b);
        telemetry.addData("Reverse BooL:", reverse);
        telemetry.addData("Intake Dir:", intakeMotor.getDirection());
        telemetry.addData("MidTake Dir:", midtakeMotor.getDirection());
        if (gamepad2.right_trigger > 0.3 && !launching2 && !launching1) {
            if (!reverse) {
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                midtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            launching2 = true;

            intakeMotor.setPower(1.0);

            midtakeMotor.setPower(1.0);
            count = 0;
        } else if (count < 40 && launching2) {
            midtakeMotor.setPower(1.0);
            intakeMotor.setPower(1.0);
        }else if (count >= 40){
            launching2 = false;
            midtakeMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
        }

        //telemetry.addData("Middle Motor Power: ", rbToggle.update(gamepad1.right_bumper) ? 0.5 : 1.0);

        countercount++;
        count++;
        telemetry.addData("Count: ", count);

        if (gamepad1.dpad_up && !actuating) {
            actuating = true;
            outTakeServo.setPosition(0.2);
            servoCount = 0;
        } else if (servoCount < 40 && actuating) {
            outTakeServo.setPosition(0.1);
        } else  if (servoCount >= 40) {
            actuating = false;
            outTakeServo.setPosition(0.43);
        }

        if (outtakePower >1.0) {
            outtakePower = 1.0;
        } else if (outtakePower<0.0) {
            outtakePower = 0.0;
        }

        servoCount++;

        //outTakeServo.setPosition(upToggle.update(gamepad1.dpad_up) ? 0.2: 0.43);
        telemetry.addData("Servo: ", outTakeServo.getPosition());

        //intakeMotor.setPower(aToggle.update(gamepad1.a) ? 1.0 : 0.0);
        //midtakeMotor.setPower(aToggle.update(gamepad1.a) ? 1.0 : 0.0);
        topMotor.setPower(bToggle.update(gamepad1.b) ? outtakePower : 0.0);
        sideMotor.setPower(bToggle.update(gamepad1.b) ? outtakePower : 0.0);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y* Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX*1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        if (slowmode) {
            motorFrL.setPower(frontLeftPower*0.25);
            motorFrR.setPower((frontRightPower)*0.25);
            motorBL.setPower(backLeftPower*0.25);
            motorBR.setPower(backRightPower*0.25);
        } else {
            motorFrL.setPower(frontLeftPower);
            motorFrR.setPower(frontRightPower);
            motorBL.setPower(backLeftPower);
            motorBR.setPower(backRightPower);
        }
        //motorFrL.setPower(frontLeftPower);
        //motorFrR.setPower((frontRightPower));
        //motorBL.setPower(backLeftPower);
        //motorBR.setPower(backRightPower);





    }


}