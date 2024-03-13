package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class RevFlyWheels extends Command {

    public RevFlyWheels() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    public RevFlyWheels(boolean isAuto) {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.resetKI();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.indexer.getShooterSensorState()) {
            Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
        }

        Robot.shooter.setupAndRev(Robot.nextShot.getShooterRPM());

        if (Robot.shooter.getShooterRPM() >= Robot.nextShot.getMinimumRPM() && !Robot.runningAutonomousMode) {
            Robot.oi.setRumble(true);
        } else {
            Robot.oi.setRumble(false);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
//        if (Robot.runningAutonomousMode && counter > 1.5*50) {
//            return true;
//        }
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // don't want to set shooter RPM to 0 because the shoot command is taking over
        // instead, use onFalse in OI and call shooter.stop()
        Robot.oi.setRumble(false);
    }
}
