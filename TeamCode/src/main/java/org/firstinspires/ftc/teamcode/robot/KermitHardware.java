package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.hardware.IMU;

public class KermitHardware {
    HardwareMap hwMap =  null;


    public Drivetrain dt = null;
    public Intake intake = null;


    public WobbleGripper WobbleGripper = null;


    public IMU imu;

    public Shooter shooter = null;




    public ElapsedTime time = new ElapsedTime();


    public KermitHardware(){


    }


    //Initializes hardware in each OpMode
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;


        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap);


        WobbleGripper = new WobbleGripper(hwMap);
        imu = new IMU(hwMap);
    }


    public void initNoGyro(HardwareMap ahwMap){
        hwMap = ahwMap;


        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);


        WobbleGripper = new WobbleGripper(hwMap);
    }
}
