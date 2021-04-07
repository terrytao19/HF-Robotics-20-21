package Dashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    //target - current is your error
    //and to send power proportional to error
    //just multiply it by a tuned constant
    //kP
    //so double pControl = kP * error;
//error is setpoint - current, and then your power is just kP * error + kI * integral(error) + kD * deriv(error)
//integral of error is just a running sum of error, and derivative of error is just the rate of change of error
    public static double GYRO_TURN_KP = 0.03;
    public static double GYRO_TURN_KI = 0;
    public static double GYRO_TURN_KD = 0.003;

    public static double Straight_KP = .08;
    public static double Straight_KI = 0;
    public static double Straight_KD = 0;
    public static double Target_Angle = 90;

    public static double MOTION_PROFILE_Kp = 0.1;
    public static double MOTION_PROFILE_Ki = 0;
    public static double MOTION_PROFILE_Kd = 0;
    public static double MOTION_PROFILE_KiMAX = 0.5;


   public static double MAX_ACCELERATION = .9;
   public static double MAX_VELOCITY = 1;
   public static double GRIPPER_GRAB_POS= .9;


}
