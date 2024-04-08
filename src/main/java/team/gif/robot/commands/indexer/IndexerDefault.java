package team.gif.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class IndexerDefault extends Command {

    boolean collectorState;
    boolean stageOneState;
    boolean shooterState;

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

        //-------- Possible States --------
        // 1. Only Collector Sensor - Run Collector and Stage 1 Indexer
        // 2. Collector Sensor and Stage 1 Sensor - Run Collector, Stage 1-2 Indexer
        // 3. Stage 1 Sensor - Run Stage 1-2 Indexer
        // 4. Stage 1 Sensor and Shooter Sensor - Nothing
        // 5. Shooter Sensor - Nothing
        // 6. Collector Sensor and Shooter Sensor - Reverse Collector and Stage 1 Indexer (?)

        // 0
        if (!Robot.sensors.collector() && !Robot.sensors.indexer() && !Robot.sensors.shooter()) {
            Robot.indexer.setIndexing(false);
        }

        // 1
        if (Robot.sensors.collector() && !Robot.sensors.indexer() && !Robot.sensors.shooter()) {
            Robot.indexer.setIndexing(true);
        }

        // 2
        if (Robot.sensors.collector() && Robot.sensors.indexer() && !Robot.sensors.shooter()) {
            Robot.indexer.setIndexing(true);
        }

        // 3
        if (!Robot.sensors.collector() && Robot.sensors.indexer() && !Robot.sensors.shooter()) {
           Robot.indexer.setIndexing(true);
        }

        // 4
        if (!Robot.sensors.collector() && Robot.sensors.indexer() && Robot.sensors.shooter()) {
            Robot.indexer.setIndexing(false);
        }

        // 5
        if (!Robot.sensors.collector() && !Robot.sensors.indexer() && Robot.sensors.shooter()) {
            Robot.indexer.setIndexing(false);
        }

        // Bot is indexing. Run the indexers
        if (Robot.indexer.isIndexing()) {
            Robot.indexer.setIndexer(Constants.Indexer.INDEXER_ONE_COLLECT_PERC, Constants.Indexer.INDEXER_TWO_COLLECT_PERC);
        } else {
            Robot.indexer.stopIndexerHard();
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
