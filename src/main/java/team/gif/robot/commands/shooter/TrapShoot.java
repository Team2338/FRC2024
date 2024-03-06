package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class TrapShoot extends Command {
    boolean stopFlyWheel;
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
        stopFlyWheel = false;
        finished = false;
        wristEngaged = false;
        counter = 0;
        Robot.wrist.PIDKill();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (counter <= (0.25 * 50)) { // run the flywheel from start to 0.25 seconds after game piece leaves shooter
            Robot.shooter.setShooterRPM(Constants.Shooter.TRAP_RPM);
        } else {
            Robot.shooter.stop();
        }
        System.out.println(counter);
        if (counter >= 3*50) {
            System.out.println("Wrist " + Robot.wrist.getPosition());
            if (Robot.wrist.getPosition() > Constants.Wrist.SETPOINT_TRAP_FINAL_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(-0.5);//0.3);
                System.out.println("final");
            } else {
                Robot.wrist.moveWristPercentPower(0);
                finished = true;
            }
        }

        // once shooterRPM gets to target, run the indexer
        if (Robot.shooter.getShooterRPM() >= (Constants.Shooter.TRAP_RPM * 0.9)) { // 0.9 provides tolerance
            Robot.indexer.setIndexer(0,Constants.Indexer.INDEXER_TWO_TRAP_PERC);
        }

        // once we no longer have the game piece, rotate the shooter mechanism
        if (!Robot.indexer.getShooterSensorState()) {
            //TODO: Stop before encoder reaches 1 and resets
//            if (counter <= (.4*50)) { // rotate the shooter for 0.5 seconds // todo consider changing to using PID
            if (Robot.wrist.getPosition() < Constants.Wrist.SETPOINT_TRAP_ABSOLUTE) {
                Robot.wrist.moveWristPercentPower(.3);//0.3);
            } else {
                Robot.wrist.moveWristPercentPower(0);
                wristEngaged = true;
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
        Robot.indexer.stopIndexerCoast();
        Robot.wrist.moveWristPercentPower(0);
        Robot.wrist.setTargetPosition(Robot.wrist.getPosition());
        Robot.wrist.PIDEnable();
        Robot.shooter.setVoltagePercent(0);
        System.out.println("complete!!");
    }
}
