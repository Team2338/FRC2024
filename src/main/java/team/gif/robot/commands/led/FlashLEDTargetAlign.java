package team.gif.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class FlashLEDTargetAlign extends Command {
    int counter;

    public FlashLEDTargetAlign() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;
        Robot.ledSubsystem.setAutoAlignFlash(true);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        if (counter < .10*50 ||
                (counter > .20*50) && (counter < 0.30*50) ||
                (counter > .40*50) && (counter < 0.50*50)) {
            Robot.ledSubsystem.setLEDOff();
        } else {
            Robot.ledSubsystem.setStageSafe();
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return counter++ > .60 * 50 ;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.ledSubsystem.setAutoAlignFlash(false);
    }
}
