package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Fundraiser")
public class Fundraiser extends OpMode {

    Servo outTakeServo;
    turretPIDF tuner;
    IMU imu;
    DcMotor motorFrL;
    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor intakeMotor;
    DcMotor midtakeMotor;
    DcMotorEx topMotor;
    DcMotorEx sideMotor;

    double lastTime;
    Toggle aToggle = new Toggle();
    Toggle bToggle = new Toggle();

    Toggle yToggle = new Toggle();

    Toggle rbToggle = new Toggle();

    Toggle aToggle2 = new Toggle();

    Toggle upToggle = new Toggle();
    Toggle bToggle2 = new Toggle();
    boolean intakeMode;

    boolean slowmode = false;

    double amount = 0.25;
    int count = 0;

    int countercount;
    boolean counterActuating;
    int servoCount = 0;

    double outtakePower = 0.55;

    boolean launching1 = false;
    boolean launching2 = false;
    boolean actuating = false;

    public enum PIDFINCREMENT {
        P {
            @Override
            public PIDFINCREMENT nextState() {
                return I;
            }
        },
        I {
            @Override
            public PIDFINCREMENT nextState() {
                return D;
            }
        },
        D {
            @Override
            public PIDFINCREMENT nextState() {
                return P;
            }
        };

        public abstract PIDFINCREMENT nextState() ;
    }

    PIDFINCREMENT pidf;

    boolean reverse = false;

    double maxOmega = 100 * Math.PI * 2.0;
    double targetOmega = amount * maxOmega;

    boolean shooting = false;


    @Override
    public void init() {

        motorFrL = hardwareMap.get(DcMotor.class,"lf");
        motorFrR = hardwareMap.get(DcMotor.class,"rf");
        motorBL = hardwareMap.get(DcMotor.class,"lr");
        motorBR = hardwareMap.get(DcMotor.class,"rr ");

        intakeMotor = hardwareMap.get(DcMotor.class, "vmotor1");
        midtakeMotor = hardwareMap.get(DcMotor.class, "vmotor2");
        topMotor = hardwareMap.get(DcMotorEx.class, "topLauncher");
        sideMotor = hardwareMap.get(DcMotorEx.class, "sideLauncher");
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrL.setDirection(DcMotorSimple.Direction.REVERSE);
        midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        tuner = new turretPIDF(topMotor, sideMotor);
        lastTime = getRuntime();
        outTakeServo = hardwareMap.get(Servo.class, "servo0");
        outTakeServo.setPosition(0.33);

        pidf = PIDFINCREMENT.P;


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD ));
        imu.initialize(parameters);

    }


    @Override
    public void loop() {


        shooting = bToggle.update(gamepad1.b);
        targetOmega = ((shooting) ? 1.0 : 0.0) * maxOmega * amount;
        tuner.update(targetOmega);


        slowmode = aToggle2.update(gamepad1.a);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;



        if (gamepad1.options) {
            imu.resetYaw();
        }
        // telemetry.addData("Commanded Velocity:, ", tuner.getCommandedVelocity());
        telemetry.addData("Target Velocity:, ", tuner.getTargetVelocity());
        telemetry.addData("Alleged Measured Velocity:, ", tuner.getMeasuredVelocity());
        telemetry.addData("Top Motor Velocity:", topMotor.getVelocity());
        telemetry.addData("Side Motor Velocity:", sideMotor.getVelocity());
        telemetry.addData("Gamepad y: ", gamepad1.y);
        telemetry.addData("Launching: ", launching2 );
        telemetry.addData("Direction of Midtake: ", midtakeMotor.getDirection());
        telemetry.addData("Outtake Amount", amount);
        telemetry.addData("PIDF: P ", tuner.getP());
        telemetry.addData("PIDF: I ", tuner.getI());
        telemetry.addData("PIDF: D ", tuner.getD());

        if (Math.abs(sideMotor.getVelocity()) > 1200 && Math.abs(topMotor.getVelocity()) > 1200) {
            gamepad1.rumbleBlips(3);
        } else {
            gamepad1.rumbleBlips(0);
        }


        if (gamepad1.dpad_up && !launching2 && !launching1) {
            launching1 = true;
            midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            midtakeMotor.setPower(0.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
            count = 0;
        } else if (count < 20 && launching1) {
            midtakeMotor.setPower(0.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
        } else if (count < 80 && launching1) {
            midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            midtakeMotor.setPower(1.0);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(1.0);
        }else if (count >= 20){
            launching1 = false;
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(0.0);
            midtakeMotor.setPower(0.0);
        }


        telemetry.addData("Reverse BooL:", reverse);
        telemetry.addData("Intake Dir:", intakeMotor.getDirection());
        telemetry.addData("MidTake Dir:", midtakeMotor.getDirection());
        if (gamepad1.right_trigger > 0.3 && !launching2 && !launching1) {

            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
           // intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
           //midtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            launching2 = true;

            intakeMotor.setPower(1.0);

            midtakeMotor.setPower(1.0);
            count = 0;
        } else if (count < 40 && launching2 && !launching1) {
            midtakeMotor.setPower(1.0);
            intakeMotor.setPower(1.0);
        }else if (count >= 40 && !launching1){
            launching2 = false;
            midtakeMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
        }



        countercount++;
        count++;
        telemetry.addData("Count: ", count);

        if (gamepad1.dpad_up && !actuating) {
            actuating = true;
            outTakeServo.setPosition(0.1);
            servoCount = 0;
        } else if (servoCount < 10 && actuating) {
            outTakeServo.setPosition(0.1);
        } else  if (servoCount >= 10) {
            actuating = false;
            outTakeServo.setPosition(0.33);
        }

        if (outtakePower >1.0) {
            outtakePower = 1.0;
        } else if (outtakePower<0.0) {
            outtakePower = 0.0;
        }

        servoCount++;


        telemetry.addData("Servo: ", outTakeServo.getPosition());

        //FLYWHEEL
        //topMotor.setPower(bToggle.update(gamepad1.b) ? outtakePower : 0.0);
        //sideMotor.setPower(bToggle.update(gamepad1.b) ? outtakePower : 0.0);

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
    }


}