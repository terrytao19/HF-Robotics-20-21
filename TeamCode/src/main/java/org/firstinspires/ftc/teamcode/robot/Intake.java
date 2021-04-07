package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intake;




    public static final double INTAKE_POWER = 0.5;
    public static final double INTAKE_POWER_SLOW = 0.25;


    public Intake(HardwareMap ahwMap) {
        intake = ahwMap.get(DcMotor.class, "intake");
        ;


        intake.setDirection(DcMotor.Direction.FORWARD);
    }


    public void on() {
        intake.setPower(INTAKE_POWER);


    }


    public void reverse() {
        intake.setPower(-INTAKE_POWER_SLOW);


    }


    public void off() {
        intake.setPower(0);


    }
}
