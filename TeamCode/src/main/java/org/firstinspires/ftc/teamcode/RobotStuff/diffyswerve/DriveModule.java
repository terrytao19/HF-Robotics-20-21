package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Misc.DataLogger;
import org.firstinspires.ftc.teamcode.Misc.PID;

import Dashboard.RobotConstants;


public class DriveModule {
    Robot robot;
    public final ModuleSide moduleSide;
    public final Vector2d positionVector;
    public final DataLogger dataLogger;
    boolean debuggingMode;

    public DcMotor motor1; //top motor
    public DcMotor motor2; //bottom motor
    //used for logic that allows robot to rotate modules as little as possible
    public boolean takingShortestPath = false;
    public boolean reversed = false;

    public static final double TICKS_PER_MODULE_REV = 8192;
    public static final double CM_WHEEL_DIAMETER = 3 * 2.54;
    public static final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public final double MAX_MOTOR_POWER = 1;
    public static final double TICKS_PER_CM = (TICKS_PER_MODULE_REV) / (CM_WHEEL_DIAMETER * Math.PI);

    // Unit vectors represent rot power v translation power coordinate system
    public Vector2d MOTOR_1_VECTOR = new Vector2d(1 / Math.sqrt(2), 1 / Math.sqrt(2)); //unit vector (MAY BE SWITCHED with below)
    public Vector2d MOTOR_2_VECTOR = new Vector2d(-1 / Math.sqrt(2), 1 / Math.sqrt(2)); //unit vector

    public double MAX_MOTOR_SPEED = 1.0;//setting to 1 may increase robot top speed, but may decrease accuracy
    //used for scaling pivot component (see getPivotComponent() method)
    public final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 60;

    //if module is within this number of degrees from its target orientation, no pivot power will be applied
    public final double ALLOWED_MODULE_ORIENTATION_ERROR = 10;

    //TODO: tune this variable (see commented out section in TeleOp)
    //was 1.7
    public double ROT_ADVANTAGE = 1.7; //max rotation power divided by max translation power (scaling factor)

    //this variable is set to 0.7 because when in RUN_USING_ENCODERS mode, powers about ~0.7 are the same
    //setting to 1 may increase robot top speed, but may decrease accuracy
    public double MAX_MOTOR_POWER = 1.0; //was 0.7

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    //PID values need to be tuned
    public PID StraightPID = new PID(RobotConstants.Straight_KP, RobotConstants.Straight_KI, RobotConstants.Straight_KD);
    public PID turnToAnglePIDTest = new PID(RobotConstants.GYRO_TURN_KP, RobotConstants.GYRO_TURN_KI, RobotConstants.GYRO_TURN_KD);
    public PID turnToAnglePID = new PID(0.03, 0, 0.003);


    public DriveModule(Robot robot, ModuleSide moduleSide, DataLogger dataLogger) {
        this.robot = robot;
        this.moduleSide = moduleSide;
        this.dataLogger = dataLogger;

        if (moduleSide == ModuleSide.RIGHT) {
            motor1 = robot.hardwareMap.dcMotor.get("rightTopMotor");
            motor2 = robot.hardwareMap.dcMotor.get("rightBottomMotor");
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            positionVector = new Vector2d((double) 18 / 2, 0); //points from robot center to right module
        } else {
            motor1 = robot.hardwareMap.dcMotor.get("leftTopMotor");
            motor2 = robot.hardwareMap.dcMotor.get("leftBottomMotor");
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            positionVector = new Vector2d((double) -18 / 2, 0); //points from robot center to left module
        }

        //set run mode to NOT use encoders for velocity PID regulation
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motors will brake when zero power is applied (rather than coast)
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /**
     * Set this module's rotation and translation speeds.
     *
     * @param steerSpeed the speed to turn the module at (change the azimuth of the wheel) in radians/sec
     * @param spinSpeed  the speed to spin the wheel at in radians/sec
     */
    private void setModuleSpeeds(double steerSpeed, double spinSpeed) {
        // (rad/s) the portion of the motors' rotation that goes towards changing the azimuth of the wheel
        double motorSteerSpeed = steerSpeed * RobotConstants.STEERING_GEAR_RATIO;
        // (rad/s) the portion of the motors' rotation that goes towards spinning the axle of the wheel (translation)
        double motorSpinSpeed = spinSpeed * RobotConstants.DRIVE_GEAR_RATIO;
        // one motor will spin at motorSteerSpeed + motorSpinSpeed, the other will spin at motorSteerSpeed - motorSpinSpeed because math
        // if the speeds will be too fast for the motors, scale down
        if (Math.abs(motorSteerSpeed) + Math.abs(motorSpinSpeed) > MAX_MOTOR_SPEED) {
            double scaleDown = MAX_MOTOR_SPEED / (Math.abs(motorSteerSpeed) + Math.abs(motorSpinSpeed));
            motorSteerSpeed *= scaleDown;
            motorSpinSpeed *= scaleDown;
        }
    }

    //this method updates the target vector for the module based on input from auto/teleop program
    public void updateTarget(Vector2d transVec, double rotMag) { //translation vector and rotation magnitude
        //converts robot heading to the angle type used by Vector2d class

        //transVec = transVec.rotate(-90); // to change to heading instead of cartesian from joystick

        //converts the translation vector from a robot centric to a field centric one
        //Vector2d transVecFC = transVec.rotateBy(robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING), Angle.Direction.COUNTER_CLOCKWISE); //was converted robot heading, was clockwise

        //TODO: this disables robot centric
        //Vector2d transVecFC = isRobotCentric ? transVec : transVec.rotateBy(robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING), Angle.Direction.COUNTER_CLOCKWISE);

        Vector2d transVecFC = transVec.rotateBy(robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING), Angle.Direction.COUNTER_CLOCKWISE);

        //vector needed to rotate robot at the desired magnitude
        //based on positionVector of module (see definition for more info)
        Vector2d rotVec = positionVector.normalize(rotMag).rotateBy(90, Angle.Direction.COUNTER_CLOCKWISE); //theoretically this should be rotated 90, not sure sure it doesn't need to be

        //combine desired robot translation and robot rotation to get goal vector for the module
        Vector2d targetVector = transVecFC.add(rotVec);

        //allows modules to reverse power instead of rotating 180 degrees
        //example: useful when going from driving forwards to driving backwards
        int directionMultiplier = -1; //was positive 1
        if (reversed) { //reverse direction of translation because module is reversed
            targetVector = targetVector.reflect();
            directionMultiplier = 1;
        }

        if (debuggingMode) {
            dataLogger.addField(transVecFC.getX());
            dataLogger.addField(transVecFC.getY());
            dataLogger.addField(rotVec.getX());
            dataLogger.addField(rotVec.getY());
            dataLogger.addField(targetVector.getX());
            dataLogger.addField(targetVector.getY());
         //   dataLogger.addField(getCurrentOrientation().getAngle());
            dataLogger.addField(reversed);
        }

        //calls method that will apply motor powers necessary to reach target vector in the best way possible, based on current position
        goToTarget(targetVector, directionMultiplier);

        if (debuggingMode) {
            robot.telemetry.addData(moduleSide + " REVERSED: ", reversed);
            robot.telemetry.addData(moduleSide + " Trans Vec FC: ", transVecFC);
            robot.telemetry.addData(moduleSide + " Rot Vec: ", rotVec);
        }
    }

    //used in place of updateTarget() when the "absolute heading mode" is being used in TeleOp
    //"absolute heading mode" means the angle of the rotation joystick is used to determine the desired angle of the robot
    public void updateTargetAbsRotation(Vector2d transVec, Angle targetHeading, double scaleFactor) { //translation vector and rotation magnitude
        Angle robotHeading = robot.getRobotHeading(); //todo: this changed
        double rotMag = getRotMag(targetHeading) * scaleFactor;
        updateTarget(transVec, rotMag);
    }

    private double getRotMag(Angle targetHeading) {
     //   robot.telemetry.addData("Difference between joystick and robot", targetHeading.getDifference(robotHeading));
       // robot.telemetry.addData("Direction from robot to joystick", robotHeading.directionTo(targetHeading));

        //    double unsignedDifference = RobotUtil.scaleVal(targetHeading.getDifference(robotHeading),15, 60, .3, 1);
        //todo: check if below line is causing problem
        //  if (robotHeading.directionTo(targetHeading) == Angle.Direction.CLOCKWISE) {
        //   return unsignedDifference * -1;
        //  } else {
        //      return unsignedDifference;
        //}
    }


    //sets motor powers for robot to best approach given target vector
    public void goToTarget(Vector2d targetVector, int directionMultiplier) {
        //how much the module needs to translate (and in which direction)
        double moveComponent = targetVector.getMagnitude() * directionMultiplier;

        //how much the module needs to pivot (change its orientation)
        double pivotComponent = 0;
        if (targetVector.getMagnitude() != 0) {

        } else {
            //if target vector is zero (joystick is within deadband) don't pivot modules
            pivotComponent = 0;
        }

        //vector in an (invented) coordinate system that represents desired (relative) module translation and module rotation
        Vector2d powerVector = new Vector2d(moveComponent, pivotComponent); //order very important here

        if (debuggingMode) {
            dataLogger.addField(powerVector.getX());
            dataLogger.addField(powerVector.getY());
        }


        if (debuggingMode) {
            robot.telemetry.addData(moduleSide + " Target Vector Angle: ", targetVector.getAngleDouble(Angle.AngleType.ZERO_TO_360_HEADING));
            robot.telemetry.addData(moduleSide + " Power Vector: ", powerVector);
           // robot.telemetry.addData(moduleSide + " Current orientation:
        }

    }


    //returns a scalar corresponding to how much power the module needs to apply to rotating
    //this is necessary because of the differential nature of a diff swerve drive
    public double getPivotComponent(Vector2d targetVector, Angle currentAngle) {
        Angle targetAngle = targetVector.getAngle();
        double angleDiff = targetAngle.getDifference(currentAngle); //number from 0 to 180 (always positive)


        if (debuggingMode)
            robot.telemetry.addData(moduleSide + " Angle diff (abs. value): ", angleDiff);
        Angle.Direction direction = currentAngle.directionTo(targetAngle);

        //CCW is negative for heading system
        //   if (rotateModuleMode == RotateModuleMode.DO_NOT_ROTATE_MODULES) return 0;

        // if (angleDiff < ALLOWED_MODULE_ORIENTATION_ERROR) {
        //don't rotate module if it's currently within x degrees of its target orientation
        //avoids constant twitching of modules
        //    return 0;
        // } else if (angleDiff > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
        //      //rotation power is maxed out if the difference is more than this angle
        //    if (direction == Angle.Direction.CLOCKWISE) return ROT_ADVANTAGE;
        //     else return -1 * ROT_ADVANTAGE;
        //  } else {
        //     //scale module rotation power based on set constants
        //     if (direction == Angle.Direction.CLOCKWISE) return angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
        //     else return -1 * angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
        //  }
    }


    //used in place of updateTarget() when the "absolute heading mode" is being used in TeleOp
    //"absolute heading mode" means the angle of the rotation joystick is used to determine the desired angle of the robot

    public void getRotMag(Angle targetHeading, Angle robotHeading) {
        robot.telemetry.addData("Difference between joystick and robot", targetHeading.getDifference(robotHeading));
        robot.telemetry.addData("Direction from robot to joystick", robotHeading.directionTo(targetHeading));
    }

    /* public void setMotorPowers (Vector2d powerVector) {

        //this is one way to convert desired ratio of module translation and module rotation to motor powers
        //vectors are not strictly necessary for this, but made it easier to visualize
        //more documentation on this visualization method coming soon
        Vector2d motor1Unscaled = powerVector.projection(MOTOR_1_VECTOR);
        Vector2d motor2Unscaled = powerVector.projection(MOTOR_2_VECTOR);

        //makes sure no vector magnitudes exceed the maximum motor power
        Vector2d[] motorPowersScaled = Vector2d.batchNormalize(MAX_MOTOR_POWER, motor1Unscaled, motor2Unscaled);
        double motor1power = motorPowersScaled[0].getMagnitude();
        double motor2power = motorPowersScaled[1].getMagnitude();

        //this is to add sign to magnitude, which returns an absolute value
        if (motorPowersScaled[0].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_1_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
            motor1power *= -1;
        }
        if (motorPowersScaled[1].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_2_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
            motor2power *= -1;
        }

        if (debuggingMode) {
            robot.telemetry.addData(moduleSide + " Motor 1 Power: ", motor1power);
            robot.telemetry.addData(moduleSide + " Motor 2 Power: ", motor2power);
        }
        motor1.setPower(motor1power);
        motor2.setPower(motor2power);

        if (debuggingMode) {
            dataLogger.addField(motor1power);
            dataLogger.addField(motor2power);
        }
*/

}