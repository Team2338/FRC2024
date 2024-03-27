package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.CollectorDefault;

public class ToggleCollectorDefault extends Command {

    public ToggleCollectorDefault() {
        super();
        addRequirements(Robot.collector,Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.collector.removeDefaultCommand();
        Robot.shooter.stop();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.setDefaultCommand(new CollectorDefault());
    }
}
