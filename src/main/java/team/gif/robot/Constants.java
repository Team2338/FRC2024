// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DEBOUNCE_DEFAULT = 0.020;

    public static final class Drivetrain { // ToDo tune - remove when done
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kRearLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kRearLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kRearRightTurningMotorReversed = false;

        public static final double FRONT_LEFT_OFFSET = 79.641015625; //79.189; //82.089; // these are off by .1 values
        public static final double REAR_LEFT_OFFSET = -136.31835937; // -137.473046875; //221.66015;//-138.25195;
        public static final double FRONT_RIGHT_OFFSET = -20.578515625; //160.40039 + 180;// + 25.31;
        public static final double REAR_RIGHT_OFFSET = 155.590625; // 155.490625; // updated with new encoder 3/7/24

        // Distance between centers of front and back wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(22.5);

        // Distance between centers of left and right wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        // TODO: get feedback from driver
        public static final double COAST_DRIVE_RPM = 2200; //3000;//2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1100; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;
    }

    public static final class ModuleConstants { // ToDo tune - remove when done
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 153.0 / 25.0; //27.0 / 4.0; on 2023 bot. // need to ask luke
        public static final double ENCODER_CPR = 2048.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double DRIVE_ENCODER_ROT_2_METER = Math.PI * WHEEL_DIAMETER_METERS / (GEAR_RATIO * ENCODER_CPR);
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;

        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID { //TODO: tuning pid
            public static final double frontLeftP = 0.55;// 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01;//0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.55;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.55;//0.35;
            public static final double rearLeftFF = 0.01;//0.01;
            public static final double rearRightP = 0.55;//0.35; // 0.6
            public static final double rearRightFF = 0.01;//0.01;
        }
    }

    public static final class DrivetrainAuto {
        public static final double kP_FORWARD = 3.0;
        public static final double kP_ROTATION = 3.0;
        public static final double MAX_MODULE_SPEED_MPS = 4;
        public static final double DRIVEBASE_RADIUS_METERS = 0.4131;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 5.0;
        public static final double PY_CONTROLLER = 5.0;
        public static final double P_THETA_CONTROLLER = 3.7;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double DRIVE_SUPER_FAST = 1.0;
        public static final double DRIVE_FAST = 0.7;
        public static final double DRIVE_MEDIUM = 0.6;
        public static final double DRIVE_SLOW = 0.27; // 0.3;
        public static final double DRIVE_SUPER_SLOW = 0.2;
        public static final double HOLD_AT_ANGLE = 0.15;
        public static final double DRIVE_TIME_DEFAULT = 1.5; // seconds until the bot gets to the charging station
    }

    // copied this from 2023
    public static final class DrivetrainMK3 {
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kRearLeftTurningMotorReversed = true;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final boolean kRearRightTurningMotorReversed = true;


        public static final double FRONT_LEFT_OFFSET = -552 - 1024;
        public static final double REAR_LEFT_OFFSET = 80 - 1024;
        public static final double FRONT_RIGHT_OFFSET =  -821 - 1024;
        public static final double REAR_RIGHT_OFFSET = -256 - 1024;

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(25);

        // Distance between front and back wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.4375);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        public static final double COAST_DRIVE_RPM = 2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1675; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstantsMK3 {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 8.16; // TODO: need to ask someone
        public static final double ENCODER_CPR = 42; //This is for DRIVE
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double DRIVE_ENCODER_ROT_2_METER = Math.PI * WHEEL_DIAMETER_METERS / (GEAR_RATIO * ENCODER_CPR);
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;

        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID {
            public static final double frontLeftP = -0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01; //pBot 0.01 all FF
            public static final double frontRightP = -0.35;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = -0.35;
            public static final double rearLeftFF = 0.01;
            public static final double rearRightP = -0.35; // 0.6
            public static final double rearRightFF = 0.01;
        }
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static final class Shooter { // ToDo tune - remove when done (tuned for 5800RPM)
        public static final int TRAP_RPM = 1000;
        public static final int MIN_SAFEGUARD_RPM = 500;

        //Pass
        public static final double RPM_PASS = 6200;//6200;
        public static final double RPM_MIN_PASS = 4000;
        public static final double FF_PASS = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_PASS = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_PASS = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Wall
        public static final double RPM_WALL = 6200;//6200;
        public static final double RPM_MIN_WALL = 4200;
        public static final double FF_WALL = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_WALL = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_WALL = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Close
        public static final double RPM_CLOSE = 6200;
        public static final double RPM_MIN_CLOSE = 4500;
        public static final double FF_CLOSE = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_CLOSE = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_CLOSE = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Near
        public static final double RPM_NEAR = 6200;//6200;
        public static final double RPM_MIN_NEAR = 5000;//4500;
        public static final double FF_NEAR = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_NEAR = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_NEAR = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Mid
        public static final double RPM_MID = 6200;//6200;
        public static final double RPM_MIN_MID = 5600;
        public static final double FF_MID = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_MID = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_MID = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Middle
        public static final double RPM_MIDDLE = 6200;
        public static final double RPM_MIN_MIDDLE = 5600;
        public static final double FF_MIDDLE = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_MIDDLE = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_MIDDLE = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Midfar
        public static final double RPM_MIDFAR = 6200;
        public static final double RPM_MIN_MIDFAR = 5600;
        public static final double FF_MIDFAR = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_MIDFAR = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_MIDFAR = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //MidMidFar
        public static final double RPM_MIDMIDFAR = 6200;
        public static final double RPM_MIN_MIDMIDFAR = 6000;
        public static final double FF_MIDMIDFAR = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_MIDMIDFAR = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_MIDMIDFAR = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Far
        public static final double RPM_FAR = 6200;
        public static final double RPM_MIN_FAR = 6000;
        public static final double FF_FAR = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_FAR = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_FAR = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Far2
        public static final double RPM_FAR2 = 6200;
        public static final double RPM_MIN_FAR2 = 6000;
        public static final double FF_FAR2 = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_FAR2 = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_FAR2 = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Far3
        public static final double RPM_FAR3 = 6200;
        public static final double RPM_MIN_FAR3 = 6000;
        public static final double FF_FAR3 = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_FAR3 = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_FAR3 = 0.000;//0.001;               // 0.0000015; // 5800 RPM

        //Amp
        public static final double RPM_AMP = 2000;
        public static final double RPM_MIN_AMP = 1900;
        public static final double FF_AMP = 0.000145; // gets to 1800
        public static final double kP_AMP = 0.00030;
        public static final double kI_AMP = 0.00000;

        //Trap - only used for testing purposes. Trap shot is handled manually through % power, not PID
        public static final double RPM_TRAP = 0;
        public static final double RPM_MIN_TRAP = 0;
        public static final double FF_TRAP = 0.00;
        public static final double kP_TRAP = 0.00;
        public static final double kI_TRAP = 0.00;

        //Auto
        public static final double RPM_AUTOSHOT = 6200;
        public static final double RPM_MIN_AUTOSHOT = 4200;
        public static final double FF_AUTOSHOT = 0.000155; // gets to 1800
        public static final double kP_AUTOSHOT = 0.00030;
        public static final double kI_AUTOSHOT = 0.00000;

        //Idle
        public static final double RPM_IDLE = 1000;//6200;
        public static final double FF_IDLE = 0.000155; // gets to 6100    // 0.000130;  // 5800 RPM
        public static final double kP_IDLE = 0.00030;//0.0100;               // 0.0006;    // 5800 RPM
        public static final double kI_IDLE = 0.000;//0.001;               // 0.0000015; // 5800 RPM

    }

    public static final class Wrist { // tuned 02/22
        public static final double INCREASE_ANGLE_PWR_PERC = 0.1;
        public static final double INCREASE_ANGLE_PWR_PERC_CALIBRATION = 0.07;//0.1;
        public static final double DECREASE_ANGLE_PWR_PERC = 0.015; // shooter falls on own now
        public static final double DECREASE_ANGLE_PWR_PERC_CALIBRATION = 0.005;//0.015;
        // FF determined by removing PID and recording needed % power to hold at 90 degrees
        // 0.05 moves the wrist, 0.04 held pretty well, .043 also works
        public static final double FF = 0.043;
        public static final double kP = 2.0; // we want to get the wrist to the desired position quickly ToDo needs to move faster to avoid note going under indexer
        public static final double kI = 0.000;
        public static final double kD = 0.0000;

        // Encoder setpoints and values
        // These are the encoder specific values
        public static final double ENCODER_OFFSET_ABSOLUTE = -0.627783203125;// this is determined either manually or via the auto-calibration
        public static final double ABSOLUTE_PER_DEGREE = 0.00647249; // 0.008333;
        // These are the values we want the bot to utilize
        public static final double KILL_LIMIT_ABSOLUTE = 0.87;
        public static final double MAX_LIMIT_ABSOLUTE = 0.80; // largest value of encoder we want to allow, needs to be < 1.0
        public static final double MIN_LIMIT_ABSOLUTE = 0.105; // lowest value of encoder we want to allow, needs to be > HARD_STOP
        public static final double HARD_STOP_ABSOLUTE = 0.10; // value of encoder at lower limit hard stop
        public static final double MIN_LIMIT_DEGREES = 28.2; // 46.2; // this is calculated during manual calibration from reading the 90 degree value, provides a relationship between Degrees and Absolute
        public static final double MAX_LIMIT_DEGREES = (MAX_LIMIT_ABSOLUTE - MIN_LIMIT_ABSOLUTE)/ABSOLUTE_PER_DEGREE+MIN_LIMIT_DEGREES; // Convert MAX_LIMIT, for reference only
        public static final double SETPOINT_FAR3_ABSOLUTE = (71.71 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE;
        public static final double SETPOINT_FAR2_ABSOLUTE = (66.57 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE;
        public static final double SETPOINT_FAR_ABSOLUTE  = (62.5 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_MIDMIDFAR_ABSOLUTE  = (60.12 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_MIDFAR_ABSOLUTE  = (58.84 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_MID_ABSOLUTE  = (56.27 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_MIDDLE_ABSOLUTE  = (53.69 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_NEAR_ABSOLUTE = (51.12 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_CLOSE_ABSOLUTE = (39.53 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_WALL_ABSOLUTE = (31.80 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_PASS_ABSOLUTE = (55 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_COLLECT_ABSOLUTE = (30.51 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_TRAP_ABSOLUTE = (102 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; //87.16 // number in degrees, value in absolute
        public static final double SETPOINT_TRAP_FINAL_ABSOLUTE = (51.12 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_AMP_ABSOLUTE = (129.65 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE; // number in degrees, value in absolute
        public static final double SETPOINT_AUTOSHOT_ABSOLUTE = SETPOINT_WALL_ABSOLUTE;
        public static final double SAFE_STAGE_DEGREES = 33;

        public static final double MIN_COLLECT = (90.0 - MIN_LIMIT_DEGREES) * ABSOLUTE_PER_DEGREE + MIN_LIMIT_ABSOLUTE;
    }

    public static final class Indexer { // ToDo tune - remove when done
        public static final double INDEXER_ONE_COLLECT_PERC = 1.0; //done
        public static final double INDEXER_TWO_COLLECT_PERC = 0.26; // 0.24 for 7:1 // 0.3; ; works w/10:1 //done

        public static final double INDEXER_TWO_SHOOT_PERC = 1.0; // 0.9; works w/10:1 indexer2 for 5800//done
        public static final double INDEXER_TWO_TRAP_PERC = 0.6;  // not tuned

        public static final double INDEXER_ONE_EJECT_PERC = 0.8;
        public static final double INDEXER_TWO_EJECT_PERC = 0.2;

        public static final double INDEXER_TWO_FF = 0.0000; //remove?
        public static final double INDEXER_TWO_kP = 0.0000; //remove?

        public static final double INDEXER_ONE_DEBOUNCE = 0.040;
    }

    public static final class Collector { // ToDo tune - remove when done
        public static final double COLLECT_PERCENT = 0.85; // done
        public static final double EJECT_PERCENT = 0.2;
        public static final double PRE_SENSOR_AMPS = 30; // The current that will trigger the collector pre sensor
    }

    public static final class Elevator {
        public static final double FF = 0.0;
        public static final double kP = 0.040;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double TICKS_PER_INCH = 2.9;
        public static final double LIMIT_MAX = 59.1;
        public static final double LIMIT_MIN = 0.0;

        public static final double TRAP_UP_POS = LIMIT_MAX;
        public static final double TRAP_UP_MIN_POS = LIMIT_MAX - (1.0*TICKS_PER_INCH);

        public static final double AMP_POS = 26.0;
        public static final double SAFE_STAGE_POS = 0.4;
        public static final double TRAP_POS = 52.500;
        public static final double HOME_POS = 0.0;
        //public static final double MIN_COLLECT = (12.0 * TICKS_PER_INCH) + LIMIT_MIN;
    }

    public static final class Climber {
        public static final double FFClimb = 0.0060;
        public static final double kPClimb = 0.00000003;
        public static final double kIClimb = 0.0;
        public static final double kDClimb = 0.0;

        public static final double FFHold = 0.0;
        public static final double kPHold = 0.5;
        public static final double kIHold = 0.0;
        public static final double kDHold = 0.0;

        public static final double TICKS_PER_INCH = 5.848; // 23.3925; // measured at 6 in
        public static final double LIMIT_MAX = 41.157; //48; //150 //163.35; is max setting 150 letting to over run
        public static final double LIMIT_MIN = -55.0;//-52.5;//-163.35; is min is correct // TODO: needs tuning

        public static final double SAFE_ELEVATOR_TRAP_UP = LIMIT_MAX - (TICKS_PER_INCH * 1.0);

        public static final double SAFE_STAGE_POS = 0.375 * TICKS_PER_INCH;
        public static final double TRAP_POS = LIMIT_MIN;
        public static final double TRAP_MOVE_ELEVATOR_POS = -15.0;

        public static final double HOME_POS = 0.0;

        public static final double CLIMBER_UP_SPEED = 0.8;
        public static final double CLIMBER_DOWN_SPEED = -0.7;
    }

    public static final class MotorTemps {
        public static final double SHOOTER_MOTOR_TEMP = 70;
        public static final double INDEXER_MOTOR_TEMP = 70;
        public static final double DRIVETRAIN_MOTOR_TEMP = 85;
        public static final double COLLECTOR_MOTOR_TEMP = 70;
        public static final double ELEVATOR_MOTOR_TEMP = 70;
        public static final double CLIMBER_MOTOR_TEMP = 70;
    }
    public static final class LED {
        public static final int NUM_LEDS_TOTAL = 6;
    }
}
