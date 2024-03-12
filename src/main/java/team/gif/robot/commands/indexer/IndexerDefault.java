package team.gif.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerDefault extends Command {

    private int midSensorCounter;

    public IndexerDefault() {
        super();
        addRequirements(Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.indexer.setIndexing(false);
        midSensorCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // begin the indexing sequence
        if (Robot.collector.getSensorState() && !Robot.indexer.isIndexing()) {
            Robot.indexer.setIndexing(true);
            Robot.indexer.setNotePassedCollector(false);
        }

        // Note is between collector and shooter sensor
        // This may occur at the end of auto and start of teleop
        // It's the "dead zone" and needs to be handled
        if (!Robot.collector.getSensorState() &&
            Robot.indexer.getStageOneSensorState() &&
            !Robot.indexer.getShooterSensorState() &&
            !Robot.indexer.isIndexing() &&
                midSensorCounter++ > 3 ) { // midSensor debounce
            Robot.indexer.setIndexing(true);
            System.out.println("Mid sensor tripped");
        } else {
            midSensorCounter = 0;
        }

        // Bot is indexing. Run the indexers
        if (Robot.indexer.isIndexing()) {
            Robot.indexer.setIndexer(Constants.Indexer.INDEXER_ONE_COLLECT_PERC, Constants.Indexer.INDEXER_TWO_COLLECT_PERC);

            // indicate if the note has passed the collector but the bot is still indexing
            // this is so the collector can decide of it needs to collect or eject
            if (!Robot.collector.getSensorState() ) {
                Robot.indexer.setNotePassedCollector(true);
            }
        }

        // Note has reached its destination at the indexer sensor
        if (Robot.indexer.getShooterSensorState()) {
            if (Robot.collector.getSensorState()) { // this means the robot has collected a second note
                Robot.indexer.setIndexer(-Constants.Indexer.INDEXER_ONE_EJECT_PERC, 0);
            } else {
                Robot.indexer.stopIndexerHard();
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
        Robot.indexer.stopIndexerCoast();
    }
}
