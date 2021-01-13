package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotStuff.DriveController;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Angle;

//Has all of the hardware for the robot
public class Robot {




    public Robot(OpMode opMode, Position startingPosition, boolean debuggingMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        //this.opMode = opMode;
        driveController = new DriveController(this, startingPosition, debuggingMode);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");
    }
    public void initIMU() {
        //this.IMUReversed = reversed;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }
    public Angle getRobotHeading() {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

//        if (IMUReversed) {
//            return new Angle(heading-180, Angle.AngleType.NEG_180_TO_180_HEADING);
//        }
        //todo: check if heading should be negative or not
        return new Angle(-heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public double getRobotHeadingDouble(Angle.AngleType type) {
        return getRobotHeading().convertAngle(type).getAngle();
    }

    //SETUP METHODS
    public void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //takes RunMode parameter
    public void setupMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode RUN_MODE) {
        motor.setDirection(direction);
        motor.setMode(RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //ANGLE METHODS
    public Orientation getAngleOrientation() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); }
    public double getCurrentAngleZ() { return getAngleOrientation().firstAngle; }
    public double getCurrentAngleY() { return getAngleOrientation().secondAngle; }
    public double getCurrentAngleX() { return getAngleOrientation().thirdAngle; }
    public static double getZAngle(Orientation o) { return o.firstAngle; }
    public static double getYAngle(Orientation o) { return o.secondAngle; }
    public static double getXAngle(Orientation o) { return o.thirdAngle; }
    public static double wrapAngle(double angle) { return angle < 0 ? angle % (2 * Math.PI) + 2 * Math.PI : angle % (2 * Math.PI); }

    //SERVOS
    /*
    public void intakeServoOpen(){
        moveServo(intakeServo1, 0);
        moveServo(intakeServo2, 1);
    }
    public void intakeServoClose(){
        moveServo(intakeServo1, 1);
        moveServo(intakeServo2, 0);
    }
    */

  /*  //MOTORS
    public void moveMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }
    public final boolean debug;
    HardwareMap hwMap = null;

   // public Drivetrain dt = null;
   public Intake intake = null;
   public Shooter shooter = null;


    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap);


    }
*/

}

