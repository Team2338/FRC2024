package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class AutonShoot extends Command {

    private int counter;

    public AutonShoot() {
        super();
        addRequirements(Robot.shooter, Robot.indexer);
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.shooter.setShooterRPM(2000);//Constants.Shooter.REV_RPM);
        if(Robot.shooter.getShooterRPM() >= 2000 * .9) {
            counter++;
            Robot.indexer.setIndexer(0, Constants.Indexer.INDEXER_TWO_SHOOT_PERC);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if(counter > 50) {
            return true;
        }
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.setVoltagePercent(0);
        Robot.indexer.stopIndexerCoast();
    }
}
