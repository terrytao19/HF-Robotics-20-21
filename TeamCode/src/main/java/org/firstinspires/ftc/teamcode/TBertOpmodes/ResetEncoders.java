package org.firstinspires.ftc.teamcode.TBertOpmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends LinearOpMode {
    Robot robot;

    public void runOpMode()  {
        robot = new Robot(this, false);

        waitForStart();

        robot.driveController.resetEncoders();
        telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();
    }
}
