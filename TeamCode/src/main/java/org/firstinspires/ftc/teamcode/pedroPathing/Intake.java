

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Intake {


    private DcMotor intakeMotor;

    private DcMotor midtakeMotor;

    public void init(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "vmotor1");
        midtakeMotor = hardwareMap.get(DcMotor.class, "vmotor2");

        midtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Intake(boolean active) {
        
    }
}
