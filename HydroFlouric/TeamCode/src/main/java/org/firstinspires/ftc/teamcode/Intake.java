//BertTeleop
//This will mostlikely be awful but it is my first time implementing it into
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Teleop;

//https://docs.google.com/presentation/d/1IsPbXQ_Xplc64lsegg5QT01OC67hG_Yo8zllR_-8ELo/edit#slide=id.g53cffde57f_0_0alcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "BertTeleop", group="Bert")

//@override
public class Intake extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;



    public void init ()
    {
        telemetry.addData("Status:","initialized") ;
        frontRight = hwMap.dcMotor.get("frontRight");
        backRight = hwMap.dcMotor.get("backRight");

        rightDrivePower = gamepad1.right_stick_x;


    }

public void loop() {
        leftDrive.setPower(-1);
        rightDrive.setPower(1);

//
}
}
