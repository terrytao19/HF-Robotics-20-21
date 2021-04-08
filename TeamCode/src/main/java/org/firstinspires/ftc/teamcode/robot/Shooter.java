package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public DcMotor shooter;




    public static final double SHOOTER_POWER = 1;


    public Shooter(HardwareMap ahwMap) {
        shooter = ahwMap.get(DcMotor.class, "shooter");
        ;


        shooter.setDirection(DcMotor.Direction.REVERSE);
    }


    public void on() {
        shooter.setPower(SHOOTER_POWER);


    }


    public void off() {
            shooter.setPower(0);


    }
}
