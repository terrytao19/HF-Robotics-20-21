package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//Has all of the hardware for the robot
public class HardwareBert {

    HardwareMap hwMap = null;

    public Drivetrain dt = null;
    public Intake intake = null;
    public Shooter shooter = null;


    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap);


    }


}

