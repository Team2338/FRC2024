package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DrivePracticeSwerve extends Command {
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public DrivePracticeSwerve() {
        this.xLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(Robot.practiceDrivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("INITIALIZED DRIVE COMMAND");
    }

    @Override
    public void execute() {
        double x = Robot.oi.driver.getLeftX();
        x = (Math.abs(x) > Constants.DriveConstants.deadband) ? x : 0;
        double y = Robot.oi.driver.getLeftY();
        y = (Math.abs(y) > Constants.DriveConstants.deadband) ? y : 0;
        double rot = Robot.oi.driver.getRightX();
        rot = (Math.abs(rot) > Constants.DriveConstants.deadband) ? rot : 0;


        x = xLimiter.calculate(x) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        y = yLimiter.calculate(y) * Constants.ModuleConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
        rot = turningLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, rot);
        SwerveModuleState[] moduleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        Robot.practiceDrivetrain.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.practiceDrivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}