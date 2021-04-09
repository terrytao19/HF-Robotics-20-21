package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Dashboard.RobotConstants;

public class Pusher {

    public Servo pusher;




    public Pusher(HardwareMap ahwMap){
        pusher = ahwMap.get(Servo.class, "Pusher");

        pusher.setPosition(0);
    }
    public void push(){
        pusher.setPosition(RobotConstants.PUSHER_GRAB_POS);
        pusher.setPosition(0);
    }

    public void reset(){
        pusher.setPosition(0);
    }
}
