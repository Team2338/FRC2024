package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.climber.AutoClimb;
import team.gif.robot.commands.climber.HomeAll;
import team.gif.robot.commands.drivetrain.AutoRotate;
import team.gif.robot.commands.drivetrain.AutoRotateStage;
import team.gif.robot.commands.climber.RaiseClimberToTop;
import team.gif.robot.commands.collector.NoteRumble;
import team.gif.robot.commands.collector.ToggleCollectorDefault;
import team.gif.robot.commands.collector.CollectorManualControl;
import team.gif.robot.commands.driveModes.EnableBoost;
import team.gif.robot.commands.driveModes.EnableRobotOrientedMode;
import team.gif.robot.commands.drivetrain.AutoStrafeStage;
import team.gif.robot.commands.drivetrain.CreateNewPigeon;
import team.gif.robot.commands.drivetrain.MoveAwaySlow;
import team.gif.robot.commands.drivetrain.MoveCloserSlow;
import team.gif.robot.commands.drivetrain.MoveLeftSlow;
import team.gif.robot.commands.drivetrain.MoveRightSlow;
import team.gif.robot.commands.elevator.MoveElevatorToBottom;
import team.gif.robot.commands.elevator.MoveElevatorToTop;
import team.gif.robot.commands.indexer.IndexerManualControl;
import team.gif.robot.commands.toggleManualControl.ToggleManualControl;
import team.gif.robot.commands.wrist.CalibrateAngle;
import team.gif.robot.commands.shooter.RevFlyWheels;
import team.gif.robot.commands.shooter.Shoot;
import team.gif.robot.commands.shooter.ForceShoot;
import team.gif.robot.commands.shooter.AmpPosition;
import team.gif.robot.commands.wrist.SetWristPos;
import team.gif.robot.commands.wrist.SetWristPosAuto;
import team.gif.robot.commands.wrist.WristAngleUp;
import team.gif.robot.commands.wrist.WristAngleDown;

public class OI {
    /*
     * Instantiate all joysticks/controllers and their buttons here
     *
     * Examples:
     * public final CommandXboxController driver = new CommandXboxController(0);
     *
     * public final Trigger dA = driver.a();
     */

    public final CommandXboxController driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER_ID);
    public final CommandXboxController aux = new CommandXboxController(RobotMap.AUX_CONTROLLER_ID);
//-    public final CommandXboxController test = new CommandXboxController(RobotMap.TEST_CONTROLLER_ID);

    public final Trigger dA = driver.a();
    public final Trigger dB = driver.b();
    public final Trigger dX = driver.x();
    public final Trigger dY = driver.y();
    public final Trigger dLBump = driver.leftBumper();
    public final Trigger dRBump = driver.rightBumper();
    public final Trigger dBack = driver.back();
    public final Trigger dStart = driver.start();
    public final Trigger dLStickBtn = driver.leftStick();
    public final Trigger dRStickBtn = driver.rightStick();
    public final Trigger dRTrigger = driver.rightTrigger();
    public final Trigger dLTrigger = driver.leftTrigger();
    public final Trigger dDPadUp = driver.povUp();
    public final Trigger dDPadRight = driver.povRight();
    public final Trigger dDPadDown = driver.povDown();
    public final Trigger dDPadLeft = driver.povLeft();
    public final Trigger dDPadDownLeft = driver.povDownLeft();

    public final Trigger aA = aux.a();
    public final Trigger aB = aux.b();
    public final Trigger aX = aux.x();
    public final Trigger aY = aux.y();
    public final Trigger aLBump = aux.leftBumper();
    public final Trigger aRBump = aux.rightBumper();
    public final Trigger aBack = aux.back();
    public final Trigger aStart = aux.start();
    public final Trigger aLStickBtn = aux.leftStick();
    public final Trigger aRStickBtn = aux.rightStick();
    public final Trigger aRTrigger = aux.rightTrigger();
    public final Trigger aLTrigger = aux.leftTrigger();
    public final Trigger aDPadUp = aux.povUp();
    public final Trigger aDPadRight = aux.povRight();
    public final Trigger aDPadDown = aux.povDown();
    public final Trigger aDPadLeft = aux.povLeft();
    public final Trigger aDPadDownLeft = aux.povDownLeft();

//    public final Trigger tA = test.a();
//    public final Trigger tB = test.b();
//    public final Trigger tX = test.x();
//    public final Trigger tY = test.y();
//    public final Trigger tLBump = test.leftBumper();
//    public final Trigger tRBump = test.rightBumper();
//    public final Trigger tBack = test.back();
//    public final Trigger tStart = test.start();
//    public final Trigger tLStickBtn = test.leftStick();
//    public final Trigger tRStickBtn = test.rightStick();
//    public final Trigger tRTrigger = test.rightTrigger();
//    public final Trigger tLTrigger = test.leftTrigger();
//    public final Trigger tDPadUp = test.povUp();
//    public final Trigger tDPadRight = test.povRight();
//    public final Trigger tDPadDown = test.povDown();
//    public final Trigger tDPadLeft = test.povLeft();

    public final Trigger collectorGamePieceSensor = new Trigger(Robot.sensors::collector);
    public final Trigger shooterGamePieceSensor = new Trigger(Robot.sensors::shooter);


    public OI() {
        DriverStation.silenceJoystickConnectionWarning(true);
        /*
        *
        * Create controller actions here
        *
        * Usages:
        * dRTrigger.whileTrue(new CollectCommand());
        * dLTrigger.onTrue(new EjectCommand());
        * dA.whileTrue(new RepeatCommand(new RapidFire());
        * aStart.onTrue(new InstantCommand(Robot.elevator::zeroEncoder).ignoringDisable(true));
        *
        * onTrue (fka whenPressed)    Init->Execute repeats until IsFinished = true->End, will not start again at Init if still held down
        * whileTrue (fka whenHeld)    Init->Execute repeats until IsFinished = true or button released->End, will not start again at Init if still held down
        * whileTrue(new RepeatCommand()) (fka whileHeld)   Init->Execute repeats until IsFinished = true or button released->End, will start again at Init if still held down
        *
        * Simple Test:
        *   aX.onTrue(new PrintCommand("aX"));
        */

        /**
         * Driver Controller
         */

        // driver controls
        dLBump.whileTrue(new EnableBoost());
//        dRStickBtn.whileTrue(new AutoAlign());
//        dB.whileTrue(new RotateClockwise());
//        dX.whileTrue(new RotateCounterClockwise());
        dRBump.whileTrue(new EnableRobotOrientedMode());
        dY.whileTrue(new CollectorManualControl());

//        dX.whileTrue(new AutoRotateStage(120).andThen(new AutoStrafeStage()));
        dX.onTrue(new CreateNewPigeon());
        dB.whileTrue(new AutoRotateStage(240).andThen(new AutoStrafeStage()));
//        dY.whileTrue(new AutoRotateStage(0).andThen(new AutoStrafeStage()));
        dA.onTrue(new AutoRotate());

        dDPadUp.and(dStart.negate()).whileTrue(new MoveCloserSlow());
        dDPadDown.and(dStart.negate()).whileTrue(new MoveAwaySlow());
        dDPadLeft.and(dStart.negate()).whileTrue(new MoveRightSlow());
        dDPadRight.and(dStart.negate()).whileTrue(new MoveLeftSlow());

//        dLTrigger.and(dRTrigger).whileTrue(new AutoClimb());
        dRTrigger.whileTrue(new RevFlyWheels());
        dRTrigger.onFalse(new InstantCommand(Robot.shooter::stopIdle));
        dLTrigger.onTrue(new Shoot().andThen(new WaitCommand(0.25).andThen(new MoveElevatorToBottom()))); //.andThen(new InstantCommand(Robot.wrist::setWristCollectPosition)));

        // calibrations
        dStart.and(dDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition).ignoringDisable(true));
        dStart.and(dDPadLeft).onTrue(new InstantCommand(Robot.elevator::resetPosition).ignoringDisable(true));
        dStart.and(dDPadRight).onTrue(new InstantCommand(Robot.climber::resetPosition).ignoringDisable(true));
        dStart.and(dDPadDown).toggleOnTrue(new ToggleCollectorDefault());
        dStart.and(dBack).onTrue(new CalibrateAngle());

        /**
         *  Aux Controller
         */

        // manual control
        aA.and(aBack.negate().and(aStart.negate())).whileTrue(new CollectorManualControl());
//        aB.and(aBack.negate()).whileTrue(new CollectorManualControl().alongWith(new IndexerManualControl())); // used when sensors fail
        aB.and(aBack.negate()).whileTrue(new InstantCommand(Robot.wrist::setNextShotPass).andThen(new InstantCommand(Robot.wrist::disableAutoAngle)).andThen(new SetWristPos())); // used when sensors fail
        aStart.and(aBack).toggleOnTrue(new ToggleManualControl());

        //wrist
        // Single press will set the next shoot position, holding will move wrist
        aDPadUp.and(aStart.negate()).whileTrue(new InstantCommand(Robot.wrist::setNextShotMid).andThen(new InstantCommand(Robot.wrist::disableAutoAngle)).andThen(new SetWristPos()));
//        aDPadRight.and(aStart.negate()).whileTrue(new InstantCommand(Robot.wrist::setWristFarPosition).andThen(new SetWristPos()));
        aDPadRight.and(aStart.negate()).whileTrue(new InstantCommand(Robot.wrist::setNextShotNear).andThen(new InstantCommand(Robot.wrist::disableAutoAngle)).andThen(new SetWristPos()));
        aDPadLeft.and(aStart.negate()).whileTrue(new InstantCommand(Robot.wrist::enableAutoAngle).andThen(new SetWristPosAuto()));
        aDPadDown.and(aStart.negate()).whileTrue(new InstantCommand(Robot.wrist::setNextShotWall).andThen(new InstantCommand(Robot.wrist::disableAutoAngle)).andThen(new SetWristPos()));

        // For testing purposes only - testing the proper angle for Amp and Trap
//        aStart.and(aDPadLeft).whileTrue(new InstantCommand(Robot.wrist::setWristAmpPosition).andThen(new SetWristPos()));
//        aStart.and(aDPadRight).whileTrue(new InstantCommand(Robot.wrist::setWristTrapPosition).andThen(new SetWristPos()));

        //shooter
        aRBump.whileTrue(new RevFlyWheels());
        aRBump.onFalse(new InstantCommand(Robot.shooter::stopIdle));
        aLBump.onTrue(new Shoot().andThen(new WaitCommand(0.25).andThen(new MoveElevatorToBottom()))); //.andThen(new InstantCommand(Robot.wrist::setWristCollectPosition)));
        aX.and(aBack.negate()).whileTrue(new ForceShoot());
        //aX.and(aBack.negate()).whileTrue(new ForceShoot());

        aRTrigger.onTrue(new AmpPosition()); // goes to position and revs flywheel
        aLTrigger.onTrue(new Shoot().andThen(new MoveElevatorToBottom().andThen(new WaitCommand(0.5).andThen(new InstantCommand(Robot.flapper::setHorizontal))))); // shoots and returns to home
//        aY.and(aBack.negate()).whileTrue(new LoadFromSource());

        aBack.and(aA).onTrue(new RaiseClimberToTop());
        aBack.and(aX).onTrue(new MoveElevatorToTop());
        aBack.and(aY).whileTrue(new AutoClimb());
//        aBack.and(aY).onTrue(new LowerClimberAndElevator());
//        aBack.and(aB).onTrue(new TrapShoot().withTimeout(5));

        aStart.and(aDPadUp).whileTrue(new WristAngleUp());
        aStart.and(aDPadDown).whileTrue(new WristAngleDown());
        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.wrist::setNextShotPass).andThen(new InstantCommand(Robot.wrist::disableAutoAngle)).andThen(new SetWristPos()));
//        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.shooter::setPipeline2));
//        aY.and(aBack.negate()).onTrue(new InstantCommand(Robot.shooter::setPipeline0));
        aY.and(aBack.negate()).onTrue(new InstantCommand(Robot.flapper::setHorizontal));
//        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.wrist::BumpAngle));

        // auto sensor actions
        collectorGamePieceSensor.debounce(Constants.DEBOUNCE_DEFAULT).onTrue(new NoteRumble().andThen(new WaitCommand(0.1).andThen(new NoteRumble())));
        collectorGamePieceSensor.debounce(Constants.DEBOUNCE_DEFAULT).onTrue(new InstantCommand(Robot.wrist::setWristCollectPosition));
        shooterGamePieceSensor.debounce(Constants.DEBOUNCE_DEFAULT).onTrue(new InstantCommand(Robot.shooter::setShooterRPMIdle));

        aA.and(aStart).and(aBack.negate()).onTrue(new HomeAll());
        // testing purposes
//        dY.whileTrue(new RaiseClimber());
//        dX.whileTrue(new LowerClimber());
    }

    public void setRumble(boolean rumble){
        driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, rumble ? 1.0 : 0.0);
        aux.getHID().setRumble(GenericHID.RumbleType.kRightRumble, rumble ? 1.0 : 0.0);
    }

    /**
     * This method provides additional protection when switching between combo buttons.
     *  aA.and(aBack.negate()).onTrue works but will call aA when aBack is released. This
     *  may not be desired, but is also not standard user behavior. The combo is typically
     *  utilized when the user is holding one button down, and then presses a second, then
     *  releases the secind and then the first.
     *  If you only want aA to be called when aBack is not active, then use this to route
     *  the desired command
     */
    void routeAButton(){
        if ( !aux.getHID().getBackButton()) {
            new PrintCommand("Just A").schedule();
        }
    }
}
