package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class LowerClimberAndElevator extends Command {
    public LowerClimberAndElevator() {
        super();
        addRequirements(Robot.elevator, Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.climber.setTargetPosition(Constants.Climber.TRAP_POS);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        if (Robot.climber.getPosition() < Constants.Climber.TRAP_MOVE_ELEVATOR_POS) {
            Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_POS);
            return true;
        }
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
}
