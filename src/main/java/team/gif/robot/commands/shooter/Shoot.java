package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Shoot extends Command {
    public Shoot() {
        super();
        addRequirements(Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        System.out.println("shooting");
//        if (Robot.shooter.getRPM() >= (Constants.Shooter.REV_RPM - 20.0)) { //allow tolerance
            Robot.indexer.setIndexer(0, Constants.Indexer.STAGE_TWO);
//        }
//        Robot.shooter.setRPM(4000);
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
