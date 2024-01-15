package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class Collector_In extends CommandBase {
    public Collector_In() {
        super();
        addRequirements(Robot.collector);
    }
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.collector.Turn(0.7);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.Turn(0.0);
    }
}
