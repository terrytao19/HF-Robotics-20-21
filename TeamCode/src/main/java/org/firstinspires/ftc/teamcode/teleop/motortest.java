package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//beyblad
@Disabled
@TeleOp
public class motortest  extends OpMode {
    DcMotor motorleft;


    public void init() {

        motorleft = hardwareMap.dcMotor.get("motor0");
    }
    public void init_loop(){}

    public void start(){}


    public void loop() {
        motorleft.setPower(1);

    }
}
