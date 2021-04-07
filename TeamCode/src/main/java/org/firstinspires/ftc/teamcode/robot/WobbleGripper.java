package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Dashboard.RobotConstants;

public class WobbleGripper {
    public Servo gripper;




    public WobbleGripper(HardwareMap ahwMap){
        gripper = ahwMap.get(Servo.class, "Wobble Gripper");

        gripper.setPosition(0);
    }
    public void grab(){
        gripper.setPosition(RobotConstants.GRIPPER_GRAB_POS);
    }


    public void reset(){
        gripper.setPosition(0);
    }
}
