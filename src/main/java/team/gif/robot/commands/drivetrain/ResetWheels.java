package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ResetWheels extends Command {
    public ResetWheels() {
        addRequirements(Robot.swerveDrivetrain);
    }

    @Override
    public void execute() {
        Robot.swerveDrivetrain.fL.resetWheel();
        Robot.swerveDrivetrain.fR.resetWheel();
        Robot.swerveDrivetrain.rL.resetWheel();
        Robot.swerveDrivetrain.rR.resetWheel();
    }
}
