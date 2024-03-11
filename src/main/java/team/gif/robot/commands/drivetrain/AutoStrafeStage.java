package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.led.FlashLEDTargetAlign;

/**
 * Used in both Autonomous and Teleop
 */

public class AutoStrafeStage extends Command {
    double xOffset;
    boolean isComplete;

    public AutoStrafeStage() {
        super();
        addRequirements(Robot.swerveDrivetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isComplete = false;
        Robot.swerveDrivetrain.setDrivePace(drivePace.COAST_RR);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        xOffset = Robot.limelightShooter.getXOffset();
        if (xOffset <= -1.0 || xOffset >= 1.0) {
            Robot.swerveDrivetrain.drive(0,(xOffset > 0 ? 1 : -1) * 0.15, 0);
        } else {
            isComplete = true;
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
    public void end(boolean interrupted) {
        Robot.swerveDrivetrain.setDrivePace(drivePace.COAST_FR);
    }
}
