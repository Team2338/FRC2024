package team.gif.robot.commands.autos.lib;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class AutonNoNote extends Command {

    double commandCounter;

    public AutonNoNote() {
        super();
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        commandCounter = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        /**
         * Returning false when there is a note in the bot keeps the command
         * running. Used in Autos to run in parallel with a path so the path can
         * be aborted when this command ends.
         *
         * Only check in the first specified seconds, After that, always return false
         * in case the bot has a note but a sensor fails during the path
         */
        if (commandCounter++ < 0.5 * 50) { // check during the initial 0.5 seconds only
            return !Robot.diagnostics.getRobotHasNote();
        } else {
            return false;
        }
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
