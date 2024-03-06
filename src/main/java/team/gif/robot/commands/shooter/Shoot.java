package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.indexer.IndexerDefault;

public class Shoot extends Command {
    boolean isFiring;
    double counter;
    double autonCounter;

    /**
     * Creates a new Shoot command. Pass in true in for auto mode.
     */
    public Shoot() {
        super();
        addRequirements(Robot.indexer,Robot.shooter);
    }

    public Shoot(boolean isAuto) {
        //Pathplanner takes control of the indexer for the entire path
        //so we remove it here so that we can have the indexer default for
        //the rest of the path
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFiring = false;
        counter = 0;
        autonCounter = 0;
        Robot.indexer.stopIndexerCoast();
        //We need to remove the default command if we are in autonomous mode
        //because the default command will fight with this command for control
        //of the indexer
        if (Robot.runningAutonomousMode) {
            Robot.indexer.removeDefaultCommand();
            Robot.indexer.getCurrentCommand().cancel();
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
        if ((Robot.shooter.getShooterRPM() >= Robot.nextShot.getMinimumRPM() || isFiring) &&
                (Robot.wrist.getPosition() >= (Robot.nextShot.getWristAngle()*.95))) { //allow tolerance
            //this may need to move down to line 48
            Robot.indexer.setIndexer(0, Constants.Indexer.INDEXER_TWO_SHOOT_PERC);
            isFiring = true;
        } else {
            Robot.shooter.setShooterRPM(Robot.nextShot.getShooterRPM());
        }

        if (isFiring) {
            counter++;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter > (0.5*50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopIndexerCoast();
        Robot.shooter.setVoltagePercent(0);
        if (Robot.indexer.getDefaultCommand() == null) {
            Robot.indexer.setDefaultCommand(new IndexerDefault());
        }
        Robot.wrist.setWristCollectPosition();
    }
}
