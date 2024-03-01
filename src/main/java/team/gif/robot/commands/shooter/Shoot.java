package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.indexer.IndexerDefault;

public class Shoot extends Command {
    boolean isFiring;
    double counter;

    public Shoot() {
        super();
        addRequirements(Robot.indexer,Robot.shooter);
    }

    public Shoot(boolean isAuto) {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFiring = false;
        counter = 0;
        Robot.indexer.stopIndexerCoast();
        if (Robot.runningAutonomousMode) {
            Robot.indexer.removeDefaultCommand();
            System.out.println("remove");
            Robot.indexer.getCurrentCommand().cancel();
        }
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.REV_RPM * .98)) { //allow tolerance
            isFiring = true;
//            System.out.println("firing "+counter);
        } else {
            Robot.shooter.setShooterRPM(Constants.Shooter.REV_RPM);
        }

        System.out.println(Robot.indexer.getCurrentCommand());

        if (isFiring) {
            counter++;
            Robot.indexer.setIndexer(0, Constants.Indexer.INDEXER_TWO_SHOOT_PERC);
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
            System.out.println("set default command");
        }
        Robot.wrist.setWristCollectPosition();
    }
}
