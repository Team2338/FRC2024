package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.climber.RaiseClimber;
import team.gif.robot.commands.collector.NoteRumble;
import team.gif.robot.commands.collector.ToggleCollectorDefault;
import team.gif.robot.commands.collector.CollectorManualControl;
import team.gif.robot.commands.driveModes.EnableBoost;
import team.gif.robot.commands.drivetrain.MoveAwaySlow;
import team.gif.robot.commands.drivetrain.MoveCloserSlow;
import team.gif.robot.commands.drivetrain.MoveLeftSlow;
import team.gif.robot.commands.drivetrain.MoveRightSlow;
import team.gif.robot.commands.drivetrain.Reset0;
import team.gif.robot.commands.indexer.FullIndexerReverse;
import team.gif.robot.commands.indexer.IndexerManualControl;
import team.gif.robot.commands.wrist.CalibrateAngle;
import team.gif.robot.commands.shooter.RevFlyWheels;
import team.gif.robot.commands.shooter.Shoot;
import team.gif.robot.commands.shooter.ForceShoot;
import team.gif.robot.commands.wrist.WristAngleUp;
import team.gif.robot.commands.wrist.WristAngleDown;
import team.gif.robot.commands.shooter.TrapShoot;

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

    public final Trigger collectorGamePieceSensor = new Trigger(Robot.collector.sensor::get);

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

        // driver controls
        dLBump.whileTrue(new EnableBoost());
//        dRStickBtn.whileTrue(new AutoAlign());
//        dB.whileTrue(new RotateClockwise());
//        dX.whileTrue(new RotateCounterClockwise());
//        dRBump.whileTrue(new EnableRobotOriented());
        dY.whileTrue(new FullIndexerReverse());

        // calibrations
        dA.and(dDPadUp).onTrue(new InstantCommand(Robot.pigeon::resetPigeonPosition));
//        dA.and(dDPadLeft).onTrue(new InstantCommand(Robot.elevator::resetPosition));
//        dA.and(dDPadRight).onTrue(new InstantCommand(Robot.climber::resetPosition));
        dA.and(dDPadDown).toggleOnTrue(new ToggleCollectorDefault());
        dA.and(dBack).onTrue(new CalibrateAngle());

        // manual control
//-        aA.whileTrue(new CollectorManualControl()); // used when limelight fails
        aA.negate().onTrue(new PrintCommand("A Pressed"));

        aB.whileTrue(new CollectorManualControl().alongWith(new IndexerManualControl())); // used when sensors fail

        //wrist
        aDPadUp.onTrue(new InstantCommand(Robot.wrist::setWristFar));
        aDPadRight.onTrue(new InstantCommand(Robot.wrist::setWristMid));
        aDPadLeft.onTrue(new InstantCommand(Robot.wrist::setWristNear));
        aDPadDown.onTrue(new InstantCommand(Robot.wrist::setWristWall));
//        aDPadDownLeft.whileTrue(new EnableAutoWrist());

        //shooter
        aRBump.whileTrue(new RevFlyWheels());
        aLBump.onTrue(new Shoot().andThen(new InstantCommand(Robot.wrist::setWristCollect)));

//        aRTrigger.onTrue(new AmpPosition()); // goes to position and revs flywheel
//        aLTrigger.onTrue(new AmpShoot().andThen(new InstantCommand(Robot.wrist::setWristCollect)); // shoots and returns to home
//        aY.whileTrue(new LoadFromSource());

        aBack.and(aA).onTrue(new PrintCommand("Back and A Pressed"));
//        aBack.and(aX).onTrue(new RaiseElevatorToTop());
//        aBack.and(aY).onTrue(new LowerClimberAndElevator());
        aBack.and(aB).onTrue(new TrapShoot().withTimeout(3));

        aStart.and(aDPadUp).whileTrue(new WristAngleUp());
        aStart.and(aDPadDown).whileTrue(new WristAngleDown());
//        aStart.and(aDPadRight).onTrue(new InstantCommand(Robot.wrist::BumpAngle));
        aX.whileTrue(new ForceShoot());

        // auto sensor actions
        collectorGamePieceSensor.onTrue(new NoteRumble().andThen(new WaitCommand(0.1).andThen(new NoteRumble())));
        collectorGamePieceSensor.onTrue(new InstantCommand(Robot.wrist::setWristCollect));

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
}
