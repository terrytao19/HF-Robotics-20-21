package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.KermitHardware;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
@Disabled
@TeleOp
public class FieldCentric extends OpMode {
    private static double JOYSTICK_DEADZONE = 0.01;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();
    int booleanIncrementer;



    KermitHardware robot = new KermitHardware();

    double lastR = 0;
    public void init(){
        robot.initNoGyro(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        booleanIncrementer = 0;
        boolean fieldCentricToggle = gamepad1.left_trigger > 0.5;
        double leftstickX = gamepad1.left_stick_x;
        double leftstickY = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //gets the distance
        double r = Math.hypot(leftstickX, leftstickY);

        //gets the robot heading and converts it to radians
        double gyroAngle = robot.imu.getHeading();
        gyroAngle *= (Math.PI / 180);


        double robotAngle = Math.atan2(leftstickY, leftstickX);

        //Holding left trigger --> field centric mode
        if(gamepad1.left_trigger > 0.5) robotAngle -= gyroAngle;

        double x = Math.cos(robotAngle);
        double y = Math.sin(robotAngle);

        double FLPower = r * (y + x);
        double FRPower = r * (y - x);
        double BLPower = r * (y - x);
        double BRPower = r * (y + x);

        FLPower += turn;
        FRPower -= turn;
        BLPower += turn;
        BRPower -= turn;

        //See if there are any powers above 1 or below -1
        ArrayList<Double> motorPowers = new ArrayList<>(Arrays.asList(FLPower, FRPower, BLPower, BRPower));
        double maxPow = Collections.max(motorPowers);

        //If so, divides each power by the largest power
        if(maxPow > 1){
            FLPower /= maxPow;
            FRPower /= maxPow;
            BLPower /= maxPow;
            BRPower /= maxPow;
        }

        robot.dt.setMotorPower(FLPower,FRPower,BLPower,BRPower);

        //Intake controls
        if(gamepad1.right_bumper) robot.intake.on();
        else if(gamepad1.left_bumper) robot.intake.reverse();
        else robot.intake.off();

        //Telemetry (shows text on the phone)
        //telemetry.addData("Robot heading: ", robot.getHeading());
        telemetry.addData("Right joystick", gamepad1.right_stick_x);
        telemetry.addData("Turn", turn);
        telemetry.addData("FLPower", FLPower);
        telemetry.addData("Left motor position", robot.dt.FLMotor.getCurrentPosition());


    }


}
