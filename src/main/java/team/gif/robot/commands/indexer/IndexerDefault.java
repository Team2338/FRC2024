package team.gif.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerDefault extends Command {

    boolean noteDetected;

    public IndexerDefault() {
        super();
        addRequirements(Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        noteDetected = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.collector.getSensorState()) {
            noteDetected = true;
        }

        if (noteDetected) {
            Robot.indexer.setIndexer(Constants.Indexer.STAGE_ONE, Constants.Indexer.STAGE_TWO);
        }

        if (Robot.indexer.getSensorState()) {
            Robot.indexer.setIndexer(0, 0);
            noteDetected = false;
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
        Robot.indexer.setIndexer(0,0);
    }
}
