package org.firstinspires.ftc.teamcode.TBertOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Misc.PID;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.DriveModule;

import Dashboard.RobotConstants;
//@Config

//public class BertLinearOpMode extends OpMode {

    //FtcDashboard dashboard;

   // public FtcDashboard getDashboard() {
   //     return dashboard;
  //  }



  // ElapsedTime time = new ElapsedTime();

   // public void initHardware(HardwareBert robot){
    //    this.robot = robot;
    //}


    // represents 1 loop of correction PID loop. Called repeatedly until the target is reached
   /* public double PIDStraightCorrection(double current, double target, PID pid){
        double error = target - current;
        //if the error is greater than 180, we know it has to cross the 180 -180 boundary, so we switch to a 0-360 scale
        //this is called an "angle wrap"
        if(Math.abs(error) > 180){
            current = (current + 360) % 360;
            target = (target + 360) % 360;
        }

        //resets the target in case it's changed
        pid.setTarget(target);

        //does calculations using PID controller
        pid.updatePID(current);

        //gets the output value, which is set in the method updatePID()
        return pid.getOutput();

*/
//}
