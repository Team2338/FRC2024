package team.gif.robot.commands.shooterAngle;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterAngleUp extends Command {
    public ShooterAngleUp() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double pos = Robot.shooter.getPosition();
        if (pos < Constants.Shooter.MAX_LIMIT) {
            if (pos > Constants.Shooter.MAX_LIMIT - .1) {
                Robot.shooter.setAnglePercentMove(.1);
            } else {
                Robot.shooter.setAnglePercentMove(.2);
            }
        }
        else {
            Robot.shooter.setAnglePercentMove(0);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.setAnglePercentMove(0);
    }
}
