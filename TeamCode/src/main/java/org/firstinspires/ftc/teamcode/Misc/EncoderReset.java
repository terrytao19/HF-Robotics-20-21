package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Robot;

// This is used before every match and it is used inorder to make sure the encoders haven't drifted( also use EVERY time the code is downloaded)
//INSTRUCTIONS:
// align modules to be facing the same direction (make sure not 180 degrees apart)
// press y button on controller 1 (configured with start+A)
// make sure telemetry encoder values are both 0
// if robot controls are inverted, repeat this process, but turn both modules 180 degrees away from where you reset them before
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Reset Encoders", group = "Utilities")
public class EncoderReset extends OpMode {
    Robot robot;

    public void init (){
        robot = new Robot(this, false);
    }

public void loop () {
        telemetry.addData("LEFT Module Orientation: ", robot.DriveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("RIGHT Module Orientation: ", robot.DriveController.moduleRight.getCurrentOrientation().getAngle());
        telemetry.update();

        if (gamepad1.y) {
            robot.driveController.resetEncoders();
        }
}
}
