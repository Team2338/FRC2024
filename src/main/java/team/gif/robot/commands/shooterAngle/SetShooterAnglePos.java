package team.gif.robot.commands.shooterAngle;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class SetShooterAnglePos extends Command {
    double targetPos;
    double currentPos;

    public SetShooterAnglePos(double targetPos) {
        super();
        addRequirements(Robot.shooter); // uncomment
        this.targetPos = targetPos;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.setTargetPosition(targetPos);

        Robot.shooter.PIDRotationMove();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return (Robot.shooter.getPosition() >= Robot.shooter.getTargetPosition());
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
