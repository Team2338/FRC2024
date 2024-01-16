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
    public static final class Drivetrain {
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

        public static final double FRONT_LEFT_OFFSET = 82.089;
        public static final double REAR_LEFT_OFFSET = -138.25195;
        public static final double FRONT_RIGHT_OFFSET = -20.3906;
        public static final double REAR_RIGHT_OFFSET = 157.85156;

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
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * Constants.ModuleConstants.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstants {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 27.0 / 4.0; // need to ask aaron
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

        public static final class DrivetrainPID {
            public static final double frontLeftP = 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.35;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.35;
            public static final double rearLeftFF = 0.01;
            public static final double rearRightP = 0.35; // 0.6
            public static final double rearRightFF = 0.01;
        }
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

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }
}
