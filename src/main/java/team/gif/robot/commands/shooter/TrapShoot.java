package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class TrapShoot extends Command {
    boolean stopFlyWheel;
    int counter;
    public TrapShoot() {
        super();
        addRequirements(Robot.shooter,Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopFlyWheel = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (counter == 0 || counter <= (.25*50)) {
            Robot.shooter.setShooterRPM(Constants.Shooter.REV_RPM);
        } else {
            Robot.shooter.setVoltage(0);
        }
        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.REV_RPM * .9)) {
            Robot.indexer.setIndexer(0,.6);
        }
        if(!Robot.indexer.getSensorState()) {
            if (counter <= (.5*50)) {
                Robot.shooter.setAnglePercent(.3);
            } else {
                Robot.shooter.setAnglePercent(0);
            }
            Robot.indexer.setIndexer(0,0);
            counter++;
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
        Robot.indexer.setIndexer(0,0);
        Robot.shooter.setAnglePercent(0);
        Robot.shooter.setVoltage(0);
    }
}
