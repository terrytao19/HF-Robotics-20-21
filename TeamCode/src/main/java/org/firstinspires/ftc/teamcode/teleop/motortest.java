package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//beyblad
@TeleOp
public class motortest  extends OpMode {
    DcMotor FLMotor;


    public void init() {

        FLMotor = hardwareMap.dcMotor.get("FLMotor");
    }
    public void init_loop(){}

    public void start(){}


    public void loop() {
        FLMotor.setPower(1);

    }
}
