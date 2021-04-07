package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Misc.PID;

import java.util.Arrays;

import Dashboard.RobotConstants;

public class Drivetrain {
    public static final double TICKS_PER_MOTOR_REV = 560; // Will change depending on encoder
    public static final double WHEEL_DIAMETER_INCHES = 3.93701; // Will change depending on wheel
    public static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double TICKS_PER_ROBOT_ROTATION = 3375;
    public static final double TICKS_PER_DEGREE = TICKS_PER_ROBOT_ROTATION / 360;

    public DcMotor FLMotor;
    public DcMotor FRMotor;
    public DcMotor BLMotor;
    public DcMotor BRMotor;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    //PID values need to be tuned
    public PID turnToAnglePIDTest = new PID(RobotConstants.GYRO_TURN_KP,RobotConstants.GYRO_TURN_KI,RobotConstants.GYRO_TURN_KD);
    //public PID turnToAnglePIDTest = new PID(RobotConstants.GYRO_TURN_PID[0], RobotConstants.GYRO_TURN_PID[1], RobotConstants.GYRO_TURN_PID[2],
    //        RobotConstants.GYRO_TURN_PID[3]);
    //public PID turnToAnglePID = new PID(0.0125,0.00001,0.0004);
    //yeeee
    public PID turnToAnglePID = new PID(0.03,0,0.003);
    public PID motionProfilePID = new PID(RobotConstants.MOTION_PROFILE_Kp,RobotConstants.MOTION_PROFILE_Ki,RobotConstants.MOTION_PROFILE_Kd,
            RobotConstants.MOTION_PROFILE_KiMAX);


    //public PID motionProfilePID = new PID(0.01,0,0);
    //Initializes motors
    public Drivetrain(HardwareMap ahwMap){
        FLMotor  = ahwMap.get(DcMotor.class, "FLMotor");
        FRMotor = ahwMap.get(DcMotor.class, "FRMotor");
        BLMotor  = ahwMap.get(DcMotor.class, "BLMotor");
        BRMotor = ahwMap.get(DcMotor.class, "BRMotor");

        setMotorPower(0,0,0,0);

        //find out which direction each motor should be set to (forward or reverse?)
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        //The motors will run without encoders on by default
        disableEncoders();

        //When the robot stops, it will stop immediately
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorPower(double fl, double fr, double bl, double br){
        FLMotor.setPower(fl);
        FRMotor.setPower(fr);
        BLMotor.setPower(bl);
        BRMotor.setPower(br);
    }

    public void setMotorPower(double[] powers){
        FLMotor.setPower(powers[0]);
        FRMotor.setPower(powers[1]);
        BLMotor.setPower(powers[2]);
        BRMotor.setPower(powers[3]);
    }

    public void resetEncoders(){
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] normalizePowers(double[] powers){
        Arrays.sort(powers);
        if(powers[3] > 1){
            powers[0] /= powers[3];
            powers[1] /= powers[3];
            powers[2] /= powers[3];
            powers[3] /= powers[3];
        }
        return powers;
    }

    public double getAveragePosition(){
        return Math.abs((FLMotor.getCurrentPosition() + FRMotor.getCurrentPosition()
                + BLMotor.getCurrentPosition() + BRMotor.getCurrentPosition()) / 4);
    }

    public void disableEncoders(){
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void enableEncoders(){
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(){
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }







}
