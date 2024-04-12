package team.gif.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class SetWristPos extends Command {

    double pressedCounter;

    /**
     * Press: Sets the wrist target position, does not move wrist <br>
     * Hold: Sets the wrist target position, moves wrist <br>
     */
    public SetWristPos() {
        super();
        addRequirements(Robot.wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.autoParamsDirtyFlag = true;
        pressedCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        // user has held button down for N seconds, move the wrist
        if (pressedCounter++ > 1.5 * 50) {
            Robot.wrist.setTargetPosition(Robot.nextShot.getWristAngle());
            return true;
        }
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
