package team.gif.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class WristAngleUp extends Command {
    public WristAngleUp() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double pos = Robot.wrist.getPosition();

        if (pos < Constants.Wrist.MAX_LIMIT_ABSOLUTE) {
            Robot.wrist.moveWristPercentPower(Constants.Wrist.INCREASE_ANGLE_PWR_PERC);
            Robot.wrist.setTargetPosition(pos);
        } else {
            Robot.wrist.holdWrist();
            Robot.wrist.setTargetPosition(Constants.Wrist.MAX_LIMIT_ABSOLUTE);
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
        double pos = Robot.wrist.getPosition();

        if (pos < Constants.Wrist.MAX_LIMIT_ABSOLUTE) {
            Robot.wrist.setTargetPosition(pos);
        } else {
            Robot.wrist.setTargetPosition(Constants.Wrist.MAX_LIMIT_ABSOLUTE);
        }

        Robot.wrist.holdWrist();
    }
}
