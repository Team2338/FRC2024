package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShootManu extends Command {
    boolean isFiring;
    double counter;

    public ShootManu() {
        super();
        addRequirements(Robot.indexer,Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFiring = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.REV_RPM * .93)) { //allow tolerance
            Robot.indexer.setIndexer(0, Constants.Indexer.STAGE_TWO_SHOOTER_PERC);
//            isFiring = true;
//        } else {
//            Robot.shooter.setShooterRPM(Constants.Shooter.REV_RPM);
//        }

//        if (isFiring) {
//            counter++;
//        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;//counter > (.5*50);
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.setIndexer(0,0);
        Robot.shooter.setVoltagePercent(0);
    }
}
