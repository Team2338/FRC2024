// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.delay;
import team.gif.lib.logging.EventFileLogger;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.SwerveDrivetrain;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.drivers.Pigeon;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private static delay chosenDelay;
    public static UiSmartDashboard uiSmartDashboard;
    private Timer elapsedTime;
    private boolean runAutoScheduler;
    public static boolean runningAutonomousMode;
    public static Pigeon pigeon;
    public static Limelight limelight;
    public static OI oi;
    public static Indexer indexer;
    public static Shooter shooter;
    public static UI ui;
    public static SwerveDrivetrain swerveDrivetrain = null;
    public static DriveSwerve driveSwerve;
    private static TelemetryFileLogger telemetryLogger;
    public static EventFileLogger eventLogger;
    public static boolean isCompBot = true;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        indexer = new Indexer();
        shooter = new Shooter();
        eventLogger = new EventFileLogger();
        eventLogger.init();

        telemetryLogger = new TelemetryFileLogger();
        addMetricsToLogger();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        elapsedTime = new Timer();
        robotContainer = new RobotContainer();

        pigeon = new Pigeon(42);

        swerveDrivetrain = new SwerveDrivetrain(telemetryLogger);
        driveSwerve = new DriveSwerve();
        swerveDrivetrain.setDefaultCommand(driveSwerve);
        swerveDrivetrain.resetHeading();

        limelight = new Limelight();

        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();

        oi = new OI();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        uiSmartDashboard.updateUI();
        chosenDelay = uiSmartDashboard.delayChooser.getSelected();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        elapsedTime.reset();
        elapsedTime.start();
        runAutoScheduler = true;
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if (runAutoScheduler && (elapsedTime.get() > (chosenDelay.getValue()))) {
            if (autonomousCommand != null) {
                autonomousCommand.schedule();
            }
            runAutoScheduler = false;
            elapsedTime.stop();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 40.0 && timeLeft >= 36.0) ||
                (timeLeft <= 25.0 && timeLeft >= 21.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void addMetricsToLogger() {
        telemetryLogger.addMetric("TimeStamp", Timer::getFPGATimestamp);
        telemetryLogger.addMetric("Driver_Left_Y", () -> -Robot.oi.driver.getLeftY());
        telemetryLogger.addMetric("Driver_Left_X", () -> Robot.oi.driver.getLeftX());
        telemetryLogger.addMetric("Driver_Angle", () -> Math.atan(-Robot.oi.driver.getLeftY() / Robot.oi.driver.getLeftX()));
        telemetryLogger.addMetric("Driver_Right_X", () -> Robot.oi.driver.getRightX());
    }
}
