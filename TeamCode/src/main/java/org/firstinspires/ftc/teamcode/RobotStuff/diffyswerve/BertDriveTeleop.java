package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Misc.ButtonToggle;

    @TeleOp(name = "BertTeleOp")

    public class BertDriveTeleop  extends OpMode {
        // instance of hardware class
        Robot robot = new Robot();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        ButtonToggle toggleX = new ButtonToggle();
        Vector2d joystick1, joystick2;
        double loopStartTime = 0;
        double loopEndTime = 0;

        double startTime = 0;
        double mathTime = 0;

        public void init(){}

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


            telemetry.addData("Robot Heading: ", robot.getRobotHeading());
            telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
            telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

            telemetry.addData("StartTime: ", startTime);
            telemetry.addData("MathTime: ", mathTime);

        }
        public void stop() {
            robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
            super.stop();
        }
}
