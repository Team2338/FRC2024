package team.gif.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.SwerveModuleMK3;
import team.gif.lib.drivePace;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveDrivetrainMK3 extends SubsystemBase {
    public static SwerveModuleMK3 fL;
    public static SwerveModuleMK3 fR;
    public static SwerveModuleMK3 rR;
    public static SwerveModuleMK3 rL;

    private static SwerveDriveOdometry odometry;
    private static drivePace drivePace;

    /**
     * Constructor for swerve drivetrain using 4 swerve modules using NEOs to drive and TalonSRX to control turning
     */
    public SwerveDrivetrainMK3() {
        super();

        //define each of our modules
        fL = new SwerveModuleMK3 (
                RobotMap.PRACTICE_FRONT_LEFT_DRIVE_ID,
                RobotMap.PRACTICE_FRONT_LEFT_TURN_ID,
                Constants.DrivetrainMK3.kFrontLeftTurningMotorReversed,
                Constants.DrivetrainMK3.kFrontLeftDriveMotorReversed,
                false,
                Constants.DrivetrainMK3.FRONT_LEFT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontLeftFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontLeftP
        );

        fR = new SwerveModuleMK3 (
                RobotMap.PRACTICE_FRONT_RIGHT_DRIVE_ID,
                RobotMap.PRACTICE_FRONT_RIGHT_TURN_ID,
                Constants.DrivetrainMK3.kFrontRightTurningMotorReversed,
                Constants.DrivetrainMK3.kFrontRightDriveMotorReversed,
                false,
                Constants.DrivetrainMK3.FRONT_RIGHT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontRightFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.frontRightP
        );

        rR = new SwerveModuleMK3 (
                RobotMap.PRACTICE_REAR_RIGHT_DRIVE_ID,
                RobotMap.PRACTICE_REAR_RIGHT_TURN_ID,
                Constants.DrivetrainMK3.kRearRightTurningMotorReversed,
                Constants.DrivetrainMK3.kRearRightDriveMotorReversed,
                false,
                Constants.DrivetrainMK3.REAR_RIGHT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearRightFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearRightP
        );

        rL = new SwerveModuleMK3 (
                RobotMap.PRACTICE_REAR_LEFT_DRIVE_ID,
                RobotMap.PRACTICE_REAR_LEFT_TURN_ID,
                Constants.DrivetrainMK3.kRearLeftTurningMotorReversed,
                Constants.DrivetrainMK3.kRearLeftDriveMotorReversed,
                false,
                Constants.DrivetrainMK3.REAR_LEFT_OFFSET,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearLeftFF,
                Constants.ModuleConstantsMK3.DrivetrainPID.rearLeftP
        );

        //for field oriented stuff. helps us estimate where the robot is on the field
        odometry = new SwerveDriveOdometry(Constants.DrivetrainMK3.DRIVE_KINEMATICS, Robot.pigeon.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

//        resetHeading();
        resetDriveEncoders();

        drivePace = drivePace.COAST_FR;

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.addDouble("FL_Rotation", fL::getDriveOutput);
        swerveTab.addDouble("FR_Rotation", fR::getDriveOutput);
        swerveTab.addDouble("RL_Rotation", rL::getDriveOutput);
        swerveTab.addDouble("RR_Rotation", rR::getDriveOutput);
    }

    public SwerveDrivetrainMK3(TelemetryFileLogger logger) {
        this();

        logger.addMetric("FL_Rotation", fL::getTurningHeading);
        logger.addMetric("FR_Rotation", fR::getTurningHeading);
        logger.addMetric("RL_Rotation", rL::getTurningHeading);
        logger.addMetric("RR_Rotation", rR::getTurningHeading);

        logger.addMetric("FL_Drive_Command", () -> fL.getDriveMotor().getAppliedOutput());
        logger.addMetric("FR_Drive_Command", () -> fR.getDriveMotor().getAppliedOutput());
        logger.addMetric("RL_Drive_Command", () -> rL.getDriveMotor().getAppliedOutput());
        logger.addMetric("RR_Drive_Command", () -> rR.getDriveMotor().getAppliedOutput());

        logger.addMetric("FL_Turn_Command", () -> fL.getTurnMotor().getMotorOutputPercent());
        logger.addMetric("FR_Turn_Command", () -> fR.getTurnMotor().getMotorOutputPercent());
        logger.addMetric("RL_Turn_Command", () -> rL.getTurnMotor().getMotorOutputPercent());
        logger.addMetric("RR_Turn_Command", () -> rR.getTurnMotor().getMotorOutputPercent());

        logger.addMetric("FL_Turn_Velocity", () -> fL.getTurnMotor().getSelectedSensorVelocity());
        logger.addMetric("FR_Turn_Velocity", () -> fR.getTurnMotor().getSelectedSensorVelocity());
        logger.addMetric("RL_Turn_Velocity", () -> rL.getTurnMotor().getSelectedSensorVelocity());
        logger.addMetric("RR_Turn_Velocity", () -> rR.getTurnMotor().getSelectedSensorVelocity());
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

    /**
     * Drive the bot with given params - always field relative
     * @param x dForward
     * @param y dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {

        SwerveModuleState[] swerveModuleStates =
                Constants.DrivetrainMK3.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        setModuleStates(swerveModuleStates);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and the RobotTrajectory Singleton, for any general use {@link SwerveDrivetrainMK3#drive(double, double, double)}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        rL.setDesiredState(desiredStates[1]);
        fR.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Field Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrainMK3} or {@link team.gif.lib.RobotTrajectory}
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.DrivetrainMK3.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
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

    public double fREncoder() {
        return fR.getRawHeading();
    }

    public double fLEncoder() {
        return fL.getRawHeading();
    }

    public double rREncoder() {
        return rR.getRawHeading();
    }

    public double rLEncoder() {
        return rL.getRawHeading();
    }

    public double fRTarget() {
        return fR.target;
    }

    public double fLTarget() {
        return fL.target;
    }

    public double rRTarget() {
        return rR.target;
    }

    public double rLTarget() {
        return rL.target;
    }

    public double fRHead() {
        return fR.getTurningHeading();
    }

    public double fLHead() {
        return fL.getTurningHeading();
    }

    public double rRHead() {
        return rR.getTurningHeading();
    }

    public double rLHead() {
        return rL.getTurningHeading();
    }

    public void resetWheels() {
        fR.resetWheel();
        fL.resetWheel();
        rR.resetWheel();
        rL.resetWheel();
    }


    /**
        * Adds debugging values to the shuffleboard. Raw Heading (Ticks), Turning Heading (Radians), and Target (Radians)
        * @param shuffleboardTabName the name of the shuffleboard tab to add the debug values to
     */
    public void enableShuffleboardDebug(String shuffleboardTabName) {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab(shuffleboardTabName);

        shuffleboardTab.addDouble("FR Raw Encoder", fR::getRawHeading);
        shuffleboardTab.addDouble("FL Raw Encoder", fL::getRawHeading);
        shuffleboardTab.addDouble("RR Raw Encoder", rR::getRawHeading);
        shuffleboardTab.addDouble("RL Raw Encoder", rL::getRawHeading);

        shuffleboardTab.addDouble("FR Head", fR::getTurningHeading);
        shuffleboardTab.addDouble("FL Head", fL::getTurningHeading);
        shuffleboardTab.addDouble("RL Head", rL::getTurningHeading);
        shuffleboardTab.addDouble("RR Head", rR::getTurningHeading);

        shuffleboardTab.addDouble("FR Target", fR::getTarget);
        shuffleboardTab.addDouble("FL Target", fL::getTarget);
        shuffleboardTab.addDouble("RL Target", rL::getTarget);
        shuffleboardTab.addDouble("RR Target", rR::getTarget);


    }

}
