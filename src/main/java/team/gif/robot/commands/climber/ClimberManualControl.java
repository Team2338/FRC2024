package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
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
        double percent = -Robot.oi.aux.getRightY();

        if (Robot.oi.aux.getHID().getLeftStickButton()) {
            Robot.climber.enableSoftLimits(false);  // allow to go past soft limits
        } else {
            Robot.climber.enableSoftLimits(true);
        }

        if (percent > -0.15 && percent < 0.15) { // creates a dead band around the joystick
            Robot.climber.move(0);

        } else {
            Robot.climber.move(0.5*percent); // climber moves much faster now, slow down the manual control
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
        Robot.climber.setTargetPosition(Robot.climber.getPosition());
        Robot.climber.enableSoftLimits(true);
    }
}
