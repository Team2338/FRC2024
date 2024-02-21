package team.gif.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class MoveElevetorPerc extends Command {
    public MoveElevetorPerc() {
        super();
        addRequirements(Robot.elevator); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.elevator.setElevatorPercent(.5);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setElevatorPercent(0);
    }
}
