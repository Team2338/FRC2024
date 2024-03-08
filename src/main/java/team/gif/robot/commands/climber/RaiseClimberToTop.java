package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.commands.collector.ToggleCollectorDefault;

public class RaiseClimberToTop extends Command {
    public RaiseClimberToTop() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // starts the full trap sequence but doesn't end the climber pid
//        Robot.climber.setDefaultCommand(new ClimberPIDControl());
        new ToggleCollectorDefault().schedule();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        Robot.climber.setTargetPosition(Constants.Climber.LIMIT_MAX);
        Robot.climber.move(1);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.climber.getPosition() > Constants.Climber.LIMIT_MAX;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.move(0.0);
    }
}
