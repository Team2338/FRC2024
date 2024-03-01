package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ClimberManualControl extends Command {

    private boolean holdNeedFirstPID;
    private double holdPIDPos;

    public ClimberManualControl() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        holdNeedFirstPID = false;
        holdPIDPos = Robot.climber.getClimberPosition();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getRightY();

        if (Robot.oi.aux.getHID().getLeftStickButton()) {
            Robot.climber.enableSoftLimits(false);  // allow to go past soft limits
//            percent = percent > 0 ? 0.2 : -0.2;     // limit speed tp 20%
        } else {
            Robot.climber.enableSoftLimits(true);
        }

        if (percent > -0.05 && percent < 0.05) {
            if( holdNeedFirstPID ) {
                holdPIDPos = Robot.climber.getClimberPosition();
                holdNeedFirstPID = false;
            }
            Robot.climber.setTargetPosition(holdPIDPos);
            Robot.climber.PIDHold();
        } else {
            holdNeedFirstPID = true;
            Robot.climber.move(percent);
        }

/*        boolean moveSlow;


        if (Robot.manualControlMode) {
            double percent = -Robot.oi.aux.getHID().getRightY(); // direction of climber

            if (percent > 0.15) {
                    Robot.climber.up(moveSlow);
            } else if (percent < -0.15) {
                    Robot.climber.down(moveSlow);
            } else {
                Robot.climber.hold();
            }
        }
*/
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return !Robot.manualControlMode;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.climber.setTargetPosition(Robot.climber.getClimberPosition());
        Robot.climber.enableSoftLimits(true);
    }
}
