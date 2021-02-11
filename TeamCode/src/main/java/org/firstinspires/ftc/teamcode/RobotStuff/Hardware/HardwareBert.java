package org.firstinspires.ftc.teamcode.RobotStuff.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.IMU;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.DriveModule;

public class HardwareBert {
    HardwareMap hwMap = null;

    public Intake intake = null;
    public DriveModule dm = null;

    public IMU imu;

    public ElapsedTime time = new ElapsedTime();


    public HardwareBert(){

    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        intake = new Intake(hwMap);
        dm = new DriveModule(hwMap);
        imu = new IMU(hwMap);

    }

    public static class Intake {
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

    public static class Shooter {
        public Shooter(HardwareMap hwMap) {
        }
    }
}
