package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorDefault extends Command {
    double counter;
    boolean limelightDetectedOrExtension;

    public CollectorDefault() {
        super();
        addRequirements(Robot.collector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        limelightDetectedOrExtension = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        boolean collect = false;

        // for when the note goes undetected under the frame, provide detection extension for a time period
        if (Robot.limelightCollector.hasTarget()) {
            limelightDetectedOrExtension = true;
            counter = 0;
        } else {
            if (limelightDetectedOrExtension) {
                counter++;
                if (counter > 0.5 * 50) {
                    limelightDetectedOrExtension = false;
                }
            }
        }

        // situations where the collector should run
        if ( (Robot.collector.getSensorState() && !Robot.indexer.getSensorState() ) ||
           (!Robot.indexer.getSensorState() && !Robot.indexer.isIndexing() && limelightDetectedOrExtension)) {
            collect = true;
        }

        // a bit of a catch-all, if a note is detected by the indexer sensor, do not collect
        if (Robot.indexer.getSensorState()) {
            collect = false;
        }

        // handle the case where a second note is immediately following
        if (Robot.indexer.isIndexing() && Robot.indexer.getNotePassedCollector()){
            collect = false;
        }

        if (collect) {
            Robot.collector.collect();
        } else {
            Robot.collector.eject();
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
        Robot.collector.stop();
    }
}
