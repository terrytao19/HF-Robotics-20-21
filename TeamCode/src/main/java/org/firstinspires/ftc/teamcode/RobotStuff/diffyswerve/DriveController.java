package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Misc.DataLogger;

import Dashboard.RobotConstants;

import static org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.DriveModule.RotateModuleMode.ROTATE_MODULES;


//this has the code for the drivetrain(Diffy) and the math that goes into it
// covert < to the speed of the motor
//power input for center wheel( convert to powers for each motor)(both motors need to create a specific power)
//use encoder to adjust motor speeds
//yeet1

enum ModuleSide {LEFT, RIGHT}

public class DriveController {

    Robot robot;

    public DriveModule moduleLeft;
    public DriveModule moduleRight;

    Position robotPosition;

    DataLogger dataLogger;
    boolean debuggingMode;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    final double WHEEL_TO_WHEEL_CM = 32.5; //in cm (was 18*2.54)

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //tolerance for robot rotation (in degrees)
    public final double ALLOWED_ROBOT_ROT_ERROR = 5; //was 3

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 50;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //minimum drive power (ever)
    //TODO: actually set this to minimum possible power
    double MIN_DRIVE_POWER = 0.3;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR = 0.2;
    public final double ROBOT_ROTATION_SCALE_FACTOR_ABS = 1;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS = 1;


    //default timeouts
    public double DEFAULT_TIMEOUT_ROT_MODULES = 750; //was 500
    public double ROTATE_ROBOT_TIMEOUT = 3000;
    public double DRIVE_TIMEOUT = 4000;



    public DriveController(Robot robot, Position startingPosition, boolean debuggingMode) {
        this.robot = robot;
        this.debuggingMode = debuggingMode;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT, debuggingMode);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT, debuggingMode);

        robotPosition = startingPosition;


    }
    //defaults to debugging mode off, starting position of 0, 0
    public DriveController(Robot robot) {
        this(robot, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), false);
    }

    //defaults to debugging mode off
    public DriveController(Robot robot, Position startingPosition) {
        this(robot, startingPosition, false);
    }


    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2, boolean absHeadingMode) {
        update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
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

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
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
    public void drive(Vector2d direction, double cmDistance, double speed, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance; //BAD :(
        double initalSpeed = speed;

        alignModules = true;

        //turns modules to correct positions for straight driving
        if (alignModules)
            rotateModules(direction, false, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && /*System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && */linearOpMode.opModeIsActive()) {

            updateTracking();
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }

            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.addData("Distance Traveled", getDistanceTraveled());
            linearOpMode.telemetry.addData("CM Distance", cmDistance);

            linearOpMode.telemetry.update();


        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //aligns modules before driving but DOES NOT fix them (modules will adjust orientation while driving)
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        drive(direction, cmDistance, speed, false, true, linearOpMode);
    }



    //speed should be scalar from 0 to 1
    public void driveWithRange(Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        if (frontSensor) stopAtDistance = stopAtDistance + 10; //account for inset into robot
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        boolean continueLoop;

        double startTime = System.currentTimeMillis();
        resetDistanceTraveled();

        do {
            //loop stop condition
            if (forward) continueLoop = 0 > stopAtDistance;
            else continueLoop = 1000000 < stopAtDistance;

            //slows down drive power in certain range
            updateTracking();
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();

        } while (continueLoop && System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive());

        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //defaults to fix modules and align modules both TRUE
    public void driveWithRange(Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, LinearOpMode linearOpMode) {
        driveWithRange(direction, stopAtDistance, forward, frontSensor, speed, timeout, true, true, linearOpMode);
    }

    //speed should be scalar from 0 to 1
    public void driveWithTimeout(Vector2d direction, double cmDistance, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance / 2.0; //BAD :(
        double startTime = System.currentTimeMillis();
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        //updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();

        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //Flips between robot and field centric
    public void setDrivingStyle(boolean toRobotCentric) {
        moduleLeft.setDrivingStyle(toRobotCentric);
        moduleRight.setDrivingStyle(toRobotCentric);
    }



    public void rotateRobot (Angle targetAngle, LinearOpMode linearOpMode) {
        double startTime = System.currentTimeMillis();
        rotateModules(Vector2d.FORWARD, false, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);
        //rotateModules
        int iterations = 0;
        boolean isNegativeRotation = robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE;

        double absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
        while (absHeadingDiff > ALLOWED_MODULE_ROT_ERROR && linearOpMode.opModeIsActive() && iterations < MAX_ITERATIONS_ROBOT_ROTATE /*&& System.currentTimeMillis() - startTime < ROTATE_ROBOT_TIMEOUT*/) {
            absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
            double rotMag = RobotUtil.scaleVal(absHeadingDiff, 0, 25, 0, RobotConstants.Power); //was max power 1 - WAS 0.4 max power

            if (robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE) {
                update(Vector2d.ZERO, -rotMag);
                if (!isNegativeRotation) iterations++;
            } else {
                update(Vector2d.ZERO, rotMag);
                if (isNegativeRotation) iterations++;
            }
            linearOpMode.telemetry.addData("Rotating ROBOT", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0);
    }
    //both modules must be within allowed error for method to return
    public void rotateModules(Vector2d direction, boolean fieldCentric, double timemoutMS, LinearOpMode linearOpMode) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngle()); //was getRealAngle() (don't ask)
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngle());
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleRight.rotateModule(direction, fieldCentric);

            linearOpMode.telemetry.addData("Rotating MODULES", "");
            linearOpMode.telemetry.update();
        } while ((moduleLeftDifference > ALLOWED_MODULE_ROT_ERROR || moduleRightDifference > ALLOWED_MODULE_ROT_ERROR) && linearOpMode.opModeIsActive() && System.currentTimeMillis() < startTime + timemoutMS);
        update(Vector2d.ZERO, 0);
        //TRACKING METHODS
        //methods for path length tracking in autonomous (only useful for driving in straight lines)


    }
    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();
    }



    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    public void setRotateModuleMode(DriveModule.RotateModuleMode rotateModuleMode) {
        moduleLeft.rotateModuleMode = rotateModuleMode;
        moduleRight.rotateModuleMode = rotateModuleMode;
    }


    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }
    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }



}
