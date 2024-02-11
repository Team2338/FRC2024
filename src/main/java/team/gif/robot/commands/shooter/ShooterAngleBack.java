package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterAngleBack extends Command {

    public ShooterAngleBack() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.shooter.getPosition() > Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE) {
            Robot.shooter.moveAnglePercentPower(-0.025);
        } else {
            Robot.shooter.moveAnglePercentPower(0);
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
        Robot.shooter.moveAnglePercentPower(0);
    }
}
