package team.gif.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.Pigeon;

public class CreateNewPigeon extends Command {
    public CreateNewPigeon() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.pigeon = new Pigeon(RobotMap.PIGEON_ID);
        System.out.println("New pigeon manually instantiated");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
