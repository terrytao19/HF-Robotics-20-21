package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*public class Intake {
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    public DistanceSensor proxLeft;

    public static final double INTAKE_POWER = 0.5;
    public static final double INTAKE_POWER_SLOW = 0.25;

    public Intake(HardwareMap ahwMap){
        intakeLeft = ahwMap.get(DcMotor.class, "IntakeLeft");
        intakeRight = ahwMap.get(DcMotor.class, "IntakeRight");

        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);

        proxLeft = ahwMap.get(DistanceSensor.class, "proxLeft");
    }

    public void on(){
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }
    public void reverse(){
        intakeLeft.setPower(-INTAKE_POWER_SLOW);
        intakeRight.setPower(-INTAKE_POWER_SLOW);
    }
    public void off(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public void deploy() throws InterruptedException{
        intakeLeft.setPower(1);
        intakeRight.setPower(1);

        Thread.sleep(200);

        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public double getDistance(){
        return proxLeft.getDistance(DistanceUnit.CM);
    }
}
*/