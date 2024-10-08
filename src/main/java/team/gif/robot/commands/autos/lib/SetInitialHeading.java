// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.commands.autos.lib;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class SetInitialHeading extends Command {

    PathPlannerPath traj;

    public SetInitialHeading(PathPlannerPath trajectory) {
        super();
        traj = trajectory;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.pigeon.resetPigeonPosition(traj.getPreviewStartingHolonomicPose().getRotation().getDegrees());
        Robot.swerveDrivetrain.resetOdometry(traj.getPreviewStartingHolonomicPose());
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() { return true; }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}
}
