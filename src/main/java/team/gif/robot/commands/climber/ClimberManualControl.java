package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ClimberManualControl extends Command {
    public ClimberManualControl() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        if (Robot.oi.aux.getHID().getRightStickButton()) {
            Robot.climber.enableSoftLimits(false);
        } else {
            Robot.climber.enableSoftLimits(true);
        }

        if (Robot.manualControlMode) {
            double percent = -Robot.oi.aux.getHID().getRightY(); // direction of climber

            if (percent > 0.15) {
                if (Robot.climber.getClimberPosition() < Constants.Climber.MAX_LIMIT/100) {
                    Robot.climber.up();
                } else {
                    Robot.climber.hold();
                }
            } else if (percent < -0.15) {
                if (Robot.climber.getClimberPosition() > Constants.Climber.MIN_LIMIT/100) {
                    Robot.climber.down();
                } else {
                    Robot.climber.hold();
                }
            } else {
                Robot.climber.hold();
            }
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.manualControlMode;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.setTargetPosition(Robot.climber.getClimberPosition());
        Robot.climber.enableSoftLimits(true);
    }
}
