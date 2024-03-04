package team.gif.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.shootParams;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class WristAnglePIDControl extends Command {

    private int counter=0;

    public WristAnglePIDControl() {
        super();
        addRequirements(Robot.wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.wrist.getPosition() < Constants.Wrist.KILL_LIMIT_ABSOLUTE) {
//            Robot.wrist.PIDWristMove(); // SO that it doesn't move until the revFlyWheel
        } else {
            // defensive code in case shooter over rotates
            Robot.wrist.PIDKill();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
