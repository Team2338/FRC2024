package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterAngleBack extends Command {

    double prevPos;
    int stallCount;

    public ShooterAngleBack() {
        super();
        addRequirements(Robot.shooter); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        prevPos = -2;     // start at a number away from 0 ... 1
        stallCount = 0;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        double pos = Robot.shooter.getPosition();

//-        if (Robot.shooter.getPosition() > Constants.Shooter.MIN_LIMIT_ABSOLUTE) {
            Robot.shooter.moveAnglePercentPower(-0.025);
//-        } else {
//-            Robot.shooter.moveAnglePercentPower(0);
//        }


//        System.out.println("Delta: " + Math.abs(prevPos - pos));
//        // if collector hasn't moved, increase stall count eventually indicating we are at hard limit
//        if (Math.abs(prevPos - pos) < (1.0 * 1 / 4096)) { // 3 = min ticks to check, 4096 is number of ticks in 1 rotation
//            stallCount++;
//            System.out.println("stall");
//        } else {
//            stallCount = 0;
//        }
//        if (stallCount == 2) {
//            System.out.println("shooter angle stalling");
//            Robot.shooter.moveAnglePercentPower(0);
////        if (Robot.shooter.getPosition() > Constants.Shooter.MIN_LIMIT_ABSOLUTE) {
//        } else {
//        }
//        prevPos = pos;
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return stallCount >= 2;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.moveAnglePercentPower(0);
    }
}
