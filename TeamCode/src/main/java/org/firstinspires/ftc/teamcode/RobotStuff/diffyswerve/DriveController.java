package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Misc.DataLogger;


//this has the code for the drivetrain(Diffy) and the math that goes into it
// covert < to the speed of the motor
//power input for center wheel( convert to powers for each motor)(both motors need to create a specific power)
//use encoder to adjust motor speeds
//yeet1

enum ModuleSide {LEFT, RIGHT}

public class DriveController {

    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;
    DataLogger dataLogger;
    boolean debuggingMode;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR = 0.2;
    public final double ROBOT_ROTATION_SCALE_FACTOR_ABS = 1;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS = 1;



    public DriveController(Robot robot, Position startingPosition, boolean debuggingMode) {
        this.robot = robot;
        this.debuggingMode = debuggingMode;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT, debuggingMode);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT, debuggingMode);

        //todo: change to parameter
        Position robotPosition = startingPosition;
    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2) {
        update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
    }

    //AUTONOMOUS METHODS
    //do NOT call in a loop

    //speed should be scalar from 0 to 1
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        //turns modules to correct positions for straight driving
        //rotateModules()
        resetDistanceTraveled();
        while (getDistanceTraveled() < cmDistance && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                //speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, 0.1, 1);
            }
            updateTracking();
            update(direction.normalize(speed), 0);

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0);
    }
    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2, boolean absHeadingMode) {

        if (absHeadingMode) {
            if (joystick1.getMagnitude() == 0)
                updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_SCALE_FACTOR_ABS);
            else
                updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS);
        } else {
            if (joystick1.getMagnitude() == 0)
                update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
            else update(joystick1, -joystick2.getX() * ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR);
        }
    }
    public void updateAbsRotation(Vector2d translationVector, Vector2d joystick2, double scaleFactor) {
        Angle targetAngle = joystick2.getAngle(); //was + .convertAngle(Angle.AngleType.NEG_180_TO_180_HEADING)
        if (joystick2.getMagnitude() > 0.1 && targetAngle.getDifference(robot.getRobotHeading()) > 3) {
            moduleLeft.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
            moduleRight.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
        } else {
            moduleLeft.updateTarget(translationVector, 0);
            moduleRight.updateTarget(translationVector, 0);
        }
    }
