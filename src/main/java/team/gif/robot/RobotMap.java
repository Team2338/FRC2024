package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    public static final int PIGEON = 0;

    //SwerveDrivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 22;
    public static final int REAR_LEFT_DRIVE_MOTOR_PORT = 21;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 20;
    public static final int REAR_RIGHT_DRIVE_MOTOR_PORT = 23;
    public static final int FRONT_LEFT_CANCODER = 6;
    public static final int FRONT_RIGHT_CANCODER = 9;
    public static final int REAR_LEFT_CANCODER = 11;
    public static final int REAR_RIGHT_CANCODER = 7;

    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 18;
    public static final int REAR_LEFT_TURNING_MOTOR_PORT = 13;
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 17;
    public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 10;
}
