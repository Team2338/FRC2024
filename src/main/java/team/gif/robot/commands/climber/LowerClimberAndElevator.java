package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class LowerClimberAndElevator extends Command {
    public LowerClimberAndElevator() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.climber.move(Constants.Climber.CLIMBER_DOWN_SPEED);
        if (Robot.climber.getPosition() < Constants.Climber.TRAP_MOVE_ELEVATOR_POS) {
            Robot.elevator.setTargetPosition(Constants.Elevator.TRAP_POS);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.climber.getPosition() < Constants.Climber.LIMIT_MIN;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.move(0);
        Robot.climber.setTargetPosition(Robot.climber.getPosition());
        Robot.climber.setDefaultCommand(new ClimberPIDControl());
    }
}
