package org.firstinspires.ftc.teamcode.RobotStuff.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.IMU;
import org.firstinspires.ftc.teamcode.RobotStuff.Intake;

public class HardwareBert {
    HardwareMap hwMap = null;

    public Intake intake = null;

    public IMU imu;

    public ElapsedTime time = new ElapsedTime();

    public HardwareBert(){

    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        intake = new Intake(hwMap);
        imu = new IMU(hwMap);

    }
}
