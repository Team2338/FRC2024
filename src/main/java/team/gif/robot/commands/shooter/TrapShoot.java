package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class TrapShoot extends Command {
    boolean stopFlyWheel;
    int counter;
    boolean finished;

    public TrapShoot() {
        super();
        addRequirements(Robot.shooter,Robot.indexer); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopFlyWheel = false;
        finished = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (counter <= (0.25 * 50)) { // run the flywheel from start to 0.25 seconds after game piece leaves shooter
            Robot.shooter.setShooterRPM(Constants.Shooter.TRAP_RPM);
        } else {
            Robot.shooter.setVoltage(0);
        }

        // once shooterRPM gets to target, run the indexer
        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.TRAP_RPM * 0.9)) { // 0.9 provides tolerance
            Robot.indexer.setIndexer(0,0.6);
        }

        // once we no longer have the game piece, rotate the shooter mechanism
        if(!Robot.indexer.getSensorState()) {
            if (counter <= (.5*50)) { // rotate the shooter for 0.5 seconds // todo consider changing to using PID
                Robot.shooter.moveRotationPercentPower(0.3);
            } else {
                Robot.shooter.moveRotationPercentPower(0);
                finished = true;
            }
            Robot.indexer.setIndexer(0,0);
            counter++;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return finished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.indexer.setIndexer(0,0);
        Robot.shooter.moveRotationPercentPower(0);
        Robot.shooter.setTargetPosition(Robot.shooter.getPosition());
        Robot.shooter.setVoltage(0);
    }
}
