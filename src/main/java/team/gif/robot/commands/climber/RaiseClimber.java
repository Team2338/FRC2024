package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class RaiseClimber extends Command {
    public RaiseClimber() {
        super();
        addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        if (Robot.climber.getClimberPosition() > Constants.Climber.MAX_LIMIT/100) {
//            Robot.climber.hold();
//        } else {
//            Robot.climber.up(false);
//        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        Robot.climber.setClimberPercent(0);
    }
}
