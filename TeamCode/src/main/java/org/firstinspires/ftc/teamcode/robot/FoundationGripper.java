package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Dashboard.RobotConstants;

public class FoundationGripper {
    public Servo gripperF;
    public Servo gripperB;

    public FoundationGripper(HardwareMap ahwMap){
        gripperF = ahwMap.get(Servo.class, "gripper_front");
        gripperB = ahwMap.get(Servo.class, "gripper_back");

        gripperF.setPosition(0);
        gripperB.setPosition(0);
    }

    public void grab(){
        gripperF.setPosition(RobotConstants.FRONT_GRIPPER_GRAB_POS);
        gripperB.setPosition(RobotConstants.BACK_GRIPPER_GRAB_POS);
    }

    public void reset(){
        gripperF.setPosition(0);
        gripperB.setPosition(0);
    }


}
