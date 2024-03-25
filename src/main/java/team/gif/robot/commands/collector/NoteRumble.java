package team.gif.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

import static team.gif.robot.Robot.oi;

public class NoteRumble extends Command {
    private int rumbleCounter;
    private int endCounter;

    public NoteRumble() {
        super();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rumbleCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        rumbleCounter++;

        if (!Robot.runningAutonomousMode) {
            oi.setRumble(true);
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return rumbleCounter > 0.3*50;  // 50 cycles in 1 second
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        oi.setRumble(false);
    }
}
