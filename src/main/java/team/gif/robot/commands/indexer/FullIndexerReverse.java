package team.gif.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class FullIndexerReverse extends Command {
    public FullIndexerReverse() {
        super();
        addRequirements(Robot.indexer, Robot.collector); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.setIndexer(-Constants.Indexer.INDEXER_TWO_EJECT_PERC,0);
        Robot.collector.reverse();
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}
