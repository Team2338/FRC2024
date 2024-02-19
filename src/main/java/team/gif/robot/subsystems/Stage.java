package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Stage extends SubsystemBase {

    public Stage(){}

    public boolean getSafe() {
        return Robot.shooter.getPosition() <= (50.0 - Constants.ShooterRotation.MIN_LIMIT_DEGREES) * Constants.ShooterRotation.ABSOLUTE_PER_DEGREE + Constants.ShooterRotation.MIN_LIMIT_ABSOLUTE;//(Robot.climber.getPosition() < 100 && Robot.elevator.getPosition() < 100 && Robot.shooter.getPosition() < 100);
    }
}
