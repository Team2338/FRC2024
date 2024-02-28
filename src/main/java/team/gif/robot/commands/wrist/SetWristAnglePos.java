package team.gif.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class SetWristAnglePos extends Command {
    double targetPos;
    double currentPos;

    public SetWristAnglePos(double targetPos) {
        super();
        addRequirements(Robot.wrist);
        this.targetPos = targetPos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.wrist.setTargetPosition(targetPos);

        Robot.wrist.PIDWristMove();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return (Robot.wrist.getPosition() >= Robot.wrist.getTargetPosition());
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
