package org.firstinspires.ftc.teamcode.RobotStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor IntakeMotor;
    public static final double INTAKE_POWER = 0.5;


    public Intake(HardwareMap ahwMap){
        IntakeMotor = ahwMap.get(DcMotor.class, "IntakeMotor");

        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void on(){
        IntakeMotor.setPower(INTAKE_POWER);
    }
    public void off(){
        IntakeMotor.setPower(0);
    }

}
