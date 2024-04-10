package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.indexer.IndexerDefault;

public class ForceShoot extends Command {
    double commandCounter;

    public ForceShoot() {
        super();
        addRequirements(Robot.indexer);
    }

    public ForceShoot(boolean isAuto) {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //We need to remove the default command if we are in autonomous mode
        //because the default command will fight with this command for control
        //of the indexer
        if (Robot.runningAutonomousMode) {
            Robot.indexer.removeDefaultCommand();
            Robot.indexer.getCurrentCommand().cancel();
        }
        commandCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.indexer.setIndexer(0, Constants.Indexer.INDEXER_TWO_SHOOT_PERC);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // in standard mode, run until operator releases button
        // in autonomous mode, utilize a timeout (currently 2.5 seconds)
        return Robot.runningAutonomousMode && commandCounter++ > 2.5*50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopIndexerCoast();
        Robot.wrist.setWristCollectPosition();
        if (Robot.indexer.getDefaultCommand() == null) {
            Robot.indexer.setDefaultCommand(new IndexerDefault());
        }
        Robot.killAutoAlign = true;
        Robot.indexer.setNotePassedCollector(false);
    }
}
