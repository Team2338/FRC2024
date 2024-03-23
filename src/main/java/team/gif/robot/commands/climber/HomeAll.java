package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class HomeAll extends Command {
    boolean commandFinished;

    public HomeAll() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.setVoltagePercent(0);
        Robot.elevator.setTargetPosition(Constants.Elevator.HOME_POS);
        Robot.wrist.setTargetPosition(Constants.Wrist.MIN_LIMIT_ABSOLUTE);
        Robot.wrist.setWristWallPosition();
        commandFinished = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (Robot.climber.getPosition() > Constants.Climber.HOME_POS) {
            Robot.climber.move(Constants.Climber.CLIMBER_DOWN_SPEED);
        } else if (Robot.climber.getPosition() < (-0.25*Constants.Climber.TICKS_PER_INCH)) {
            Robot.climber.move(0.8*Constants.Climber.CLIMBER_UP_SPEED); // slow down climber up so it doesn't over shot
        } else {
            commandFinished = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return commandFinished;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.move(0.0);
    }
}
