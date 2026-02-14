package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Leave Space")
public class redHalf extends OpMode {

    IMU imu;
    DcMotor motorFrL;

    DcMotor motorFrR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor intakeMotor;
    DcMotor midtakeMotor;
    DcMotor outakeMotor;




    @Override
    public void init() {

        motorFrL = hardwareMap.get(DcMotor.class,"lf");
        motorFrR = hardwareMap.get(DcMotor.class,"rf");
        motorBL = hardwareMap.get(DcMotor.class,"lr");
        motorBR = hardwareMap.get(DcMotor.class,"rr ");

        intakeMotor = hardwareMap.get(DcMotor.class, "vmotor1");
        midtakeMotor = hardwareMap.get(DcMotor.class, "vmotor2");
        //outakeMotor = hardwareMap.get(DcMotor.class, "vmotor3");

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrR.setDirection(DcMotorSimple.Direction.REVERSE);
        midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT ));
        imu.initialize(parameters);



    }

    @Override
    public void loop() {

        motorFrL.setPower(0.2);
        motorFrR.setPower(0.2);
        motorBL.setPower(0.2);
        motorBR.setPower(0.2);

    }
}
