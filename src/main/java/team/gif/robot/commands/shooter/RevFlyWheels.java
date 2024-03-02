package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class RevFlyWheels extends Command {
    public RevFlyWheels() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.resetKI();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.wrist.setTargetPosition(Robot.wrist.getTargetPosition());
//        Robot.wrist.setWristNearPosition(); // ToDo temp test - need to combine postion with rev flywheel to reduce buttons required
        Robot.shooter.setShooterRPM(Constants.Shooter.REV_RPM);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // don't want to set shooter RPM to 0 because the shoot command is taking over
        // instead, use onFalse in OI and call shooter.stop()
    }
}
