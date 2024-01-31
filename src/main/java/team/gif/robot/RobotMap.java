package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

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
    public static final int PIGEON = 61;

    // MK3 RobotMap
    public static final int PRACTICE_REAR_LEFT_DRIVE_ID = 45;//1;
    public static final int PRACTICE_REAR_LEFT_TURN_ID = 7;//21;
    public static final int PRACTICE_REAR_RIGHT_DRIVE_ID = 14; //20;
    public static final int PRACTICE_REAR_RIGHT_TURN_ID = 8;//9;
    public static final int PRACTICE_FRONT_LEFT_DRIVE_ID = 1;//45;
    public static final int PRACTICE_FRONT_LEFT_TURN_ID = 21;//7;
    public static final int PRACTICE_FRONT_RIGHT_DRIVE_ID = 20;//14;
    public static final int PRACTICE_FRONT_RIGHT_TURN_ID = 9;//8;

    //shooter
    public static final int SHOOTER = 34;

    //indexer
    public static final int STAGE_ONE = 32;
    public static final int STAGE_TWO = 33;
    public static final int SENSOR_INDEXER_ID = 9;

    //collector
    public static final int COLLECTOR_ID = 31;
    public static final int SENSOR_COLLECTOR_ID = 0;

}
