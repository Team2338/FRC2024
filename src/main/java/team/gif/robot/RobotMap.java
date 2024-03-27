package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    //SwerveDrivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11; // Falcon 500 and Talon Fx
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 21;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 22;
    public static final int FRONT_LEFT_CANCODER_ID = 15;
    public static final int FRONT_RIGHT_CANCODER_ID = 25;
    public static final int REAR_LEFT_CANCODER_ID = 16;
    public static final int REAR_RIGHT_CANCODER_ID = 26;

    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 13; //Neo and Spark
    public static final int REAR_LEFT_TURNING_MOTOR_PORT = 14;
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 23;
    public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 24;

    public static final int PIGEON_ID = 36;

    // MK3 RobotMap
    public static final int PRACTICE_REAR_LEFT_DRIVE_ID = 45;//1;
    public static final int PRACTICE_REAR_LEFT_TURN_ID = 7;//21;
    public static final int PRACTICE_REAR_RIGHT_DRIVE_ID = 14; //20;
    public static final int PRACTICE_REAR_RIGHT_TURN_ID = 8;//9;
    public static final int PRACTICE_FRONT_LEFT_DRIVE_ID = 1;//45;
    public static final int PRACTICE_FRONT_LEFT_TURN_ID = 21;//7;
    public static final int PRACTICE_FRONT_RIGHT_DRIVE_ID = 20;//14;
    public static final int PRACTICE_FRONT_RIGHT_TURN_ID = 9;//8;
    public static final int PIGEON_PBOT_ID = 61;

    //shooter
    public static final int SHOOTER_ID = 29;

    // wrist
    public static final int WRIST_ID = 50;
    public static final int WRIST_ENCODER_ID = 53;

    //indexer
    public static final int STAGE_ONE_ID = 32;
    public static final int STAGE_TWO_ID = 33;
    public static final int SHOOTER_SENSOR_PORT = 2;
    public static final int MIDDLE_SENSOR_PORT = 1;

    //collector
    public static final int COLLECTOR_ID = 31;
    public static final int SENSOR_COLLECTOR_PORT  = 0;

    //Elevator
    public static final int ELEVATOR_ID = 51;

    //Climber
    public static final int CLIMBER_ID = 52;

    //LEDs
    public static final int LED_PWM_PORT = 9;
    public static final int[] NOTE_LEDS = {0,5};
    public static final int[] STAGE_LEDS = {1,2,3,4};
}
