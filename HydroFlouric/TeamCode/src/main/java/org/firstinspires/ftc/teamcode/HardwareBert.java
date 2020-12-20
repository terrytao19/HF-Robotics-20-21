package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
// Hardware for the robot


public DcMotor  frontLeft   = null;
public DcMotor  frontRight  = null;
public DcMotor  backleft    = null;
public DcMotor  backRight   = null;
public DcMotor  wobbleArm   = null;
public DcMotor  intake      = null;
public DcMotor  shooter     = null;



public class HardwareBert {

    HardwareMap hwMap =  null;

    private ElapsedTime time  = new ElapsedTime();
    /* Constructor */


    }
    public static final double WHEEL_DIAMETER_INCHES = 3; //Used for circumference and PID
    public static final double COUNTS_PER_MOTOR_REV = 8192; // Through bore encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.387755102;
    public static final double COUNTSP_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "front_left");  //make sure motors are named this in phone
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backLeft   = hwMap.get(DcMotor.class, "back_left");
        backRight  = hwMap.get(DcMotor.class, "back_right");
        wobbleArm  = hwMap.get(DcMotor.class, "left_arm");
        intake     = hwMap.get(DcMotor.class, "intake");
        shooter    = hwMap.get(DcMotor.class, "shooter");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        wobbleArm.setPower(0);
        intake1.setPower(0);
        shooter.setPower(0);


        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS); // running encoders
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // Define and initialize ALL installed servos.
        // leftClaw  = hwMap.get(Servo.class, "left_hand");
        // rightClaw = hwMap.get(Servo.class, "right_hand");
        // leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);


    }
}
