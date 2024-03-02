package team.gif.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class AutoRotate extends Command {
    double xOffset;
    boolean isComplete;
    public AutoRotate() {
        super();
//        addRequirements(Robot.swerveDrivetrain); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isComplete = false;
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

        double pGain = 0.012;
        double ffGain = 0.10;

        if (Robot.limelightShooter.hasTarget()) {
            xOffset = Robot.limelightShooter.getXOffset();
            if (xOffset < -2.0 || xOffset > 2.0) {
//                System.out.println( (xOffset>0?-1:1)*(ffGain+pGain*Math.abs(xOffset)));
                System.out.println(xOffset);
                Robot.swerveDrivetrain.drive(0,0,(xOffset>0?-1:1)*(ffGain+pGain*Math.abs(xOffset)));
            } else {
                isComplete = true;
                System.out.println("done");
                Robot.swerveDrivetrain.drive(0,0,0);
            }
        } else {
            // ToDo need to add time based
            isComplete = true;
        }
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return isComplete;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
