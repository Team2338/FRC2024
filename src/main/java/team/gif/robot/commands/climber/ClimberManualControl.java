package team.gif.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ClimberManualControl extends Command {

//    private boolean holdNeedFirstPID;
//    private double holdPIDPos;

    public ClimberManualControl() {
        super();
        addRequirements(Robot.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
//        holdNeedFirstPID = false;
//        holdPIDPos = Robot.climber.getPosition();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double percent = -Robot.oi.aux.getRightY();

        if (Robot.oi.aux.getHID().getLeftStickButton()) {
            Robot.climber.enableSoftLimits(false);  // allow to go past soft limits
        } else {
            Robot.climber.enableSoftLimits(true);
        }

        if (percent > -0.15 && percent < 0.15) { // creates a dead band around the joystick
//            if( holdNeedFirstPID ) {
//                holdPIDPos = Robot.climber.getPosition();
//                holdNeedFirstPID = false;
//            }
//            Robot.climber.setTargetPosition(holdPIDPos);
//            Robot.climber.PIDHold();
            Robot.climber.move(0);

        } else {
//            holdNeedFirstPID = true;
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
        Robot.climber.setTargetPosition(Robot.climber.getPosition());
        Robot.climber.enableSoftLimits(true);
    }
}
