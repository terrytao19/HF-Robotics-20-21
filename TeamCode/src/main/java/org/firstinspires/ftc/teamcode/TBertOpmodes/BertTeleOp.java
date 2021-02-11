package org.firstinspires.ftc.teamcode.TBertOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.RobotStuff.Hardware.HardwareBert;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Angle;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Vector2d;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp(name = "BertTeleOp")
public class BertTeleOp  extends OpMode {
    // instance of hardware class
    HardwareBert robot = new HardwareBert();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    ButtonToggle toggleX = new ButtonToggle();
    Vector2d joystick1, joystick2;
    double loopStartTime = 0;
    double loopEndTime = 0;

    double startTime = 0;
    double mathTime = 0;

    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop() {

        /**
         * CONTROLS:
         *
         * Left joystick: moving around
         * Right joystick: turning in place
         * Right trigger: slow mode
         *
         * Right bumper: move intake
         * Left bumper: reverse intake
         *
         * Y: move lift up
         * A: move lift down
         *
         * Dpad up: move lift to the set position above the current one
         * Dpad down: move lift to the set position below the current one
         *
         * Left stick button: reset lift
         */
        loopStartTime = System.currentTimeMillis();

        mathTime = (loopStartTime-startTime)/1000;
        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);
        joystick1 = new Vector2d(Math.cos(mathTime), Math.sin(mathTime)); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick


        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        //telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

        //telemetry.addData("Left Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
       // telemetry.addData("Right Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());

        telemetry.addData("StartTime: ", startTime);
        telemetry.addData("MathTime: ", mathTime);

    }

}
