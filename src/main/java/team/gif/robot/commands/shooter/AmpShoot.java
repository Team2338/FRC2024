package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.indexer.IndexerDefault;

public class AmpShoot extends Command {

    boolean isFiring;
    int counter;

    public AmpShoot() {
        super();
        addRequirements(Robot.shooter,Robot.wrist,Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        isFiring = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
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
        return counter > .25 * 50;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.stopIndexerCoast();
        Robot.shooter.setVoltagePercent(0);
        Robot.wrist.setWristCollectPosition();
        Robot.elevator.setTargetPosition(Constants.Elevator.SAFE_STAGE_POS);
    }
}
