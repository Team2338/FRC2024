package team.gif.robot.commands.autos;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.led.FlashLEDTargetAlign;

/**
 * Used in both Autonomous and Teleop
 */

public class AutoRotateStage extends Command {
    double xOffset;
    boolean isComplete;
    private SlewRateLimiter turnLimiter;
    double targetDegree;

    public AutoRotateStage(double targetDegree) {
        super();
//        addRequirements(Robot.swerveDrivetrain); // uncomment
        this.targetDegree = targetDegree;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isComplete = false;
        turnLimiter = new SlewRateLimiter(Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double rot;

        xOffset = Robot.pigeon.get360Heading() - targetDegree;
        if (xOffset <= -1.0 || xOffset >= 1.0) {
            rot = Robot.driveSwerve.limelightRotateMath(xOffset);
            rot = turnLimiter.calculate(rot) * Constants.ModuleConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
            Robot.swerveDrivetrain.drive(0,0,rot);
        } else {
            isComplete = true;
            new FlashLEDTargetAlign().schedule();
            Robot.swerveDrivetrain.drive(0, 0, 0);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isComplete;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
