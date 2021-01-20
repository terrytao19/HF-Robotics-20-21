package org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;


import org.firstinspires.ftc.teamcode.Misc.DataLogger;


public class DriveModule{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    Robot robot;
    public final ModuleSide moduleSide;
    public final Vector2d positionVector;
    public final DataLogger dataLogger;
    boolean debuggingMode;
    public DcMotor motor1; //top motor
    public DcMotor motor2; //bottom motor

    public final double TICKS_PER_MODULE_REV = 8192;
    public final double DEGREES_PER_TICK = 360/TICKS_PER_MODULE_REV;
    public final double CM_WHEEL_DIAMETER = 3 * 2.54;
    public final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public final double MAX_MOTOR_POWER = 1;

    // Unit vectors represent rot power v translation power coordinate system
    public Vector2d MOTOR_1_VECTOR = new Vector2d(1/Math.sqrt(2), 1/Math.sqrt(2)); //unit vector (MAY BE SWITCHED with below)
    public Vector2d MOTOR_2_VECTOR = new Vector2d(-1/Math.sqrt(2), 1/Math.sqrt(2)); //unit vector


    public DriveModule(Robot robot, ModuleSide moduleSide, DataLogger dataLogger ) {
        this.robot = robot;
        this.moduleSide = moduleSide;
        this.dataLogger = dataLogger;

        if (moduleSide == ModuleSide.RIGHT) {
            motor1 = (DcMotor) robot.hardwareMap.dcMotor.get("rightTopMotor");
            motor2 = (DcMotor) robot.hardwareMap.dcMotor.get("rightBottomMotor");
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            positionVector = new Vector2d((double) 18 / 2, 0); //points from robot center to right module
        } else {
            motor1 = (DcMotor) robot.hardwareMap.dcMotor.get("leftTopMotor");
            motor2 = (DcMotor) robot.hardwareMap.dcMotor.get("leftBottomMotor");
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




    public void setMotorPowers (Vector2d powerVector) {

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
    }

}