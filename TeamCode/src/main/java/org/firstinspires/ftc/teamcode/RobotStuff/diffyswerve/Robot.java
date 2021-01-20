package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Misc.IMU;

//Has all of the hardware for the robot
public class Robot {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;
    BNO055IMU imu;
    public DriveController driveController;

    public Robot(OpMode opMode, Position startingPosition, boolean isAuto, boolean debuggingMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
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
    public Angle Robot(OpMode opMode, Position startingPosition, boolean isAuto, boolean debuggingMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        driveController = new DriveController(this, startingPosition, debuggingMode);
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



}

