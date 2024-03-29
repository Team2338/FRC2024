package team.gif.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.lib.drivePace;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.SwerveModuleMK4;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveDrivetrain extends SubsystemBase {
    public static SwerveModuleMK4 fL;
    public static SwerveModuleMK4 fR;
    public static SwerveModuleMK4 rR;
    public static SwerveModuleMK4 rL;

    private static SwerveDriveOdometry odometry;
    private static drivePace drivePace;

    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public SwerveDrivetrain() {
        super();

        fL = new SwerveModuleMK4 (
                RobotMap.FRONT_LEFT_DRIVE_MOTOR_ID,
                RobotMap.FRONT_LEFT_TURNING_MOTOR_PORT,
                true,
                true,
                false,
                Constants.Drivetrain.FRONT_LEFT_OFFSET,
                RobotMap.FRONT_LEFT_CANCODER_ID,
                Constants.ModuleConstants.DrivetrainPID.frontLeftFF,
                Constants.ModuleConstants.DrivetrainPID.frontLeftP
        );

        fR = new SwerveModuleMK4 (
                RobotMap.FRONT_RIGHT_DRIVE_MOTOR_ID,
                RobotMap.FRONT_RIGHT_TURNING_MOTOR_PORT,
                true,
                true,
                false,
                Constants.Drivetrain.FRONT_RIGHT_OFFSET,
                RobotMap.FRONT_RIGHT_CANCODER_ID,
                Constants.ModuleConstants.DrivetrainPID.frontRightFF,
                Constants.ModuleConstants.DrivetrainPID.frontRightP
        );

        rR = new SwerveModuleMK4 (
                RobotMap.REAR_RIGHT_DRIVE_MOTOR_ID,
                RobotMap.REAR_RIGHT_TURNING_MOTOR_PORT,
                true,
                true,
                false,
                Constants.Drivetrain.REAR_RIGHT_OFFSET,
                RobotMap.REAR_RIGHT_CANCODER_ID,
                Constants.ModuleConstants.DrivetrainPID.rearRightFF,
                Constants.ModuleConstants.DrivetrainPID.rearRightP
        );

        rL = new SwerveModuleMK4 (
                RobotMap.REAR_LEFT_DRIVE_MOTOR_ID,
                RobotMap.REAR_LEFT_TURNING_MOTOR_PORT,
                true,
                true,
                false,
                Constants.Drivetrain.REAR_LEFT_OFFSET,
                RobotMap.REAR_LEFT_CANCODER_ID,
                Constants.ModuleConstants.DrivetrainPID.rearLeftFF,
                Constants.ModuleConstants.DrivetrainPID.rearLeftP
        );

        odometry = new SwerveDriveOdometry(Constants.Drivetrain.DRIVE_KINEMATICS, Robot.pigeon.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

//        resetHeading();
        resetDriveEncoders();

        drivePace = drivePace.COAST_FR;

        if(Robot.fullDashboard) {
            enableShuffleboardDebug("Swerve");
        }

        //Autos stuff
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.DrivetrainAuto.kP_FORWARD, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.DrivetrainAuto.kP_ROTATION, 0.0, 0.0), // Rotation PID constants
                        Constants.DrivetrainAuto.MAX_MODULE_SPEED_MPS, // Max module speed, in m/s
                        Constants.DrivetrainAuto.DRIVEBASE_RADIUS_METERS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if( alliance.isPresent() ){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                 },
                this
        );
    }

    public SwerveDrivetrain(TelemetryFileLogger logger) {
        this();

//        logger.addMetric("FL_Rotation", fL::getTurningHeading);
//        logger.addMetric("FR_Rotation", fR::getTurningHeading);
//        logger.addMetric("RL_Rotation", rL::getTurningHeading);
//        logger.addMetric("RR_Rotation", rR::getTurningHeading);
//
//        logger.addMetric("FL_Drive_Command", () -> fL.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("FR_Drive_Command", () -> fR.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("RL_Drive_Command", () -> rL.getDriveMotor().getDutyCycle().getValueAsDouble());
//        logger.addMetric("RR_Drive_Command", () -> rR.getDriveMotor().getDutyCycle().getValueAsDouble());
//
//        logger.addMetric("FL_Turn_Command", () -> fL.getTurnMotor().getAppliedOutput());
//        logger.addMetric("FR_Turn_Command", () -> fR.getTurnMotor().getAppliedOutput());
//        logger.addMetric("RL_Turn_Command", () -> rL.getTurnMotor().getAppliedOutput());
//        logger.addMetric("RR_Turn_Command", () -> rR.getTurnMotor().getAppliedOutput());
//
//        logger.addMetric("FL_Turn_Velocity", () -> fL.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("FR_Turn_Velocity", () -> fR.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("RL_Turn_Velocity", () -> rL.getTurnMotor().getEncoder().getVelocity());
//        logger.addMetric("RR_Turn_Velocity", () -> rR.getTurnMotor().getEncoder().getVelocity());
    }

    /**
     * Periodic function
     * - constantly update the odometry
     */
    @Override
    public void periodic() {
        odometry.update(
            Robot.pigeon.getRotation2d(),
            getPosition()
        );

        //TODO SwerveAuto can remove after PID constants are finalized and autos are running well
//        System.out.println(  "X "+ String.format("%3.2f", Robot.swervetrain.getPose().getX()) +
//                           "  Y "+ String.format("%3.2f", Robot.swervetrain.getPose().getY()) +
//                           "  R "+ String.format("%3.2f", Robot.swervetrain.getPose().getRotation().getDegrees()));
    }

    /**
     * Reset the odometry to a given pose
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeed() {
        // Example module states
        var frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.encoderDegrees()));
        var frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.encoderDegrees()));
        var rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.encoderDegrees()));
        var rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.encoderDegrees()));

// Convert to chassis speeds
        return Constants.Drivetrain.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeftState, frontRightState, rearLeft, rearRight);
    }

    /**
     * Drive the bot with given params - always field relative
     * @param x dForward
     * @param y dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        setModuleStates(swerveModuleStates);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and the RobotTrajectory Singleton, for any general use {@link SwerveDrivetrain#drive(double, double, double)}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Field Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrain} or {@link team.gif.lib.RobotTrajectory}
     */
    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Reset the position of each of the wheels so that they all are pointing straight forward
     */
    public void resetEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * Reset the pigeon heading
     */
    public void resetHeading() {
        Robot.pigeon.resetPigeonPosition();
    }


    /**
     * Get the pigeon heading
     * @return The pigeon heading in degrees
     */
    public Rotation2d getHeading() {
        return Robot.pigeon.getRotation2d();
    }

    /**
     * Get the current pose of the robot
     * @return The current pose of the robot (Pose2D)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Stop all of the modules
     */
    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    /**
     * Get the current position of each of the swerve modules
     * @return An array in form fL -> fR -> rL -> rR of each of the module positions
     */
    public SwerveModulePosition[] getPosition() {

        return new SwerveModulePosition[] {fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    /**
     * Reset the drive encoders
     */
    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * Get the current heading of the robot
     * @return the heading of the robot in degrees
     */
    public double getRobotHeading() {
        return Robot.pigeon.getCompassHeading();
    }

    /**
     * set the drivePace settings for the drivebase
     * @param drivePace the drivePace to set
     */
    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }

    /**
     * Get the current drivePace settings
     * @return the current drivePace settings
     */
    public static drivePace getDrivePace() {
        return drivePace;
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    public void enableShuffleboardDebug(String shuffleboardTabName) {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab(shuffleboardTabName);

        shuffleboardTab.addDouble("FR Heading", fR::getTurningHeadingDegrees).withPosition(2,0).withWidget(BuiltInWidgets.kGyro);
        shuffleboardTab.addDouble("FL Heading", fL::getTurningHeadingDegrees).withPosition(0,0).withWidget(BuiltInWidgets.kGyro);
        shuffleboardTab.addDouble("RR Heading", rR::getTurningHeadingDegrees).withPosition(2,2).withWidget(BuiltInWidgets.kGyro);
        shuffleboardTab.addDouble("RL Heading", rL::getTurningHeadingDegrees).withPosition(0,2).withWidget(BuiltInWidgets.kGyro);

        shuffleboardTab.addDouble("FR Raw Degrees", fR::encoderDegrees).withPosition(5,0);
        shuffleboardTab.addDouble("FL Raw Degrees", fL::encoderDegrees).withPosition(4,0);
        shuffleboardTab.addDouble("RR Raw Degrees", rR::encoderDegrees).withPosition(5,1);
        shuffleboardTab.addDouble("RL Raw Degrees", rL::encoderDegrees).withPosition(4,1);

        shuffleboardTab.addDouble("FR Raw Encoder", fR::getRawHeading).withPosition(9,0);
        shuffleboardTab.addDouble("FL Raw Encoder", fL::getRawHeading).withPosition(8,0);
        shuffleboardTab.addDouble("RR Raw Encoder", rR::getRawHeading).withPosition(9,1);
        shuffleboardTab.addDouble("RL Raw Encoder", rL::getRawHeading).withPosition(8,1);


        shuffleboardTab.addDouble("FR Raw Radians", fR::getTurningHeading).withPosition(7,0);
        shuffleboardTab.addDouble("FL Raw Radians", fL::getTurningHeading).withPosition(6,0);
        shuffleboardTab.addDouble("RR Raw Radians", rR::getTurningHeading).withPosition(7,1);
        shuffleboardTab.addDouble("RL Raw Radians", rL::getTurningHeading).withPosition(6,1);

        //TODO: Add target to shuffleboard
    }
}
