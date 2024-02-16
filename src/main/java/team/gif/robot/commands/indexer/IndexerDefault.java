package team.gif.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerDefault extends Command {

    public IndexerDefault() {
        super();
        addRequirements(Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.indexer.setIndexing(false);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // begin the indexing sequence
        if (Robot.collector.getSensorState() && !Robot.indexer.isIndexing()) {
            Robot.indexer.setIndexing(true);
            Robot.indexer.setNotePassedCollector(false);
        }

        // Bot is indexing. Run the indexers
        if (Robot.indexer.isIndexing()) {
            Robot.indexer.setIndexer(Constants.Indexer.STAGE_ONE, Constants.Indexer.STAGE_TWO);

            // indicate if the note has passed the collector but the bot is still indexing
            // this is so the collector can decide of it needs to collect or eject
            if (!Robot.collector.getSensorState() ) {
                Robot.indexer.setNotePassedCollector(true);
            }
        }

        // Note has reached it's destination at the indexer sensor
        if (Robot.indexer.getSensorState()) {
            if (Robot.collector.getSensorState()) { // this means the robot has collected a second note
                Robot.indexer.setIndexer(-0.4, 0);
            } else {
                Robot.indexer.setIndexer(0, 0);
            }
            Robot.indexer.setIndexing(false);
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
