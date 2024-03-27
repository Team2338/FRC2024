package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ClimberManualControl extends Command {

    boolean currentSoftLimitEnable; // need to store current softLimit state (see below)

    public ClimberManualControl() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentSoftLimitEnable = true; // initially the limits are enabled meaning user can't go past limit
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getRightY();

        // calling enableSoftLimits every time is time costly and causes command overruns
        // only call when user presses or releases button
        boolean newSoftLimitEnable = !Robot.oi.aux.getHID().getLeftStickButton();

        if (currentSoftLimitEnable != newSoftLimitEnable) {
            Robot.climber.enableSoftLimits(newSoftLimitEnable);  // allow to go past soft limits when false
            currentSoftLimitEnable = newSoftLimitEnable;
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
