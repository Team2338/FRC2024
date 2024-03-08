package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class TrapShoot extends Command {
    int counter;
    boolean finished;
    boolean wristEngaged;

    public TrapShoot() {
        super();
        addRequirements(Robot.shooter,Robot.wrist,Robot.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
        wristEngaged = false;
        counter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        // run the flywheel from start to 0.25 seconds after game piece leaves shooter
        if (counter <= (0.25 * 50)) {
            Robot.shooter.setShooterRPM(Constants.Shooter.TRAP_RPM);
        } else {
            Robot.shooter.stop();
        }

        // after being held for N seconds, rotate the wrist back to a safer position
        if (wristEngaged && counter++ >= 1*50) {
            if (Robot.wrist.getPosition() > Constants.Wrist.SETPOINT_TRAP_FINAL_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(-0.10);
            } else {
                // wrist is at desired position, set power to zero and end
                Robot.wrist.moveWristPercentPower(0);
                finished = true;
            }
        }

        // once shooterRPM gets to target, run the indexer
        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.TRAP_RPM * 0.9)) { // 0.9 provides tolerance
            Robot.indexer.setIndexer(0,Constants.Indexer.INDEXER_TWO_TRAP_PERC);
        }

        // once the robot no longer has the game piece, rotate the wrist to "trap"
        if (!Robot.indexer.getShooterSensorState() && !wristEngaged) {
            // rotate until desired position
            if (Robot.wrist.getPosition() < Constants.Wrist.SETPOINT_TRAP_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(.3);//0.3);
            } else {
                // reached desired position
                Robot.wrist.moveWristPercentPower(0);
                wristEngaged = true;
                Robot.wrist.setTargetPosition(Robot.wrist.getPosition());
            }
            Robot.indexer.stopIndexerCoast();
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
        Robot.wrist.setTargetPosition(Robot.wrist.getPosition());
        Robot.shooter.setVoltagePercent(0);
        System.out.println("Game over! You did GREAT!!!");
    }
}
