// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.lib.logging.EventFileLogger;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.robot.commands.climber.ClimberHold;
import team.gif.robot.commands.climber.ClimberPIDControl;
import team.gif.robot.commands.collector.CollectorDefault;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.commands.indexer.IndexerDefault;
import team.gif.robot.commands.led.LEDSubsystemDefault;
import team.gif.robot.commands.wrist.WristAnglePIDControl;
import team.gif.robot.subsystems.Climber;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.Diagnostics;
import team.gif.robot.subsystems.LEDSubsystem;
import team.gif.robot.subsystems.SwerveDrivetrain;
import team.gif.robot.subsystems.Collector;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.SwerveDrivetrainMK3;
import team.gif.robot.subsystems.Wrist;
import team.gif.robot.subsystems.drivers.Limelight;
import team.gif.robot.subsystems.drivers.Pigeon;
import team.gif.robot.commands.drivetrainPbot.DrivePracticeSwerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Command autonomousCommand;

    private RobotContainer robotContainer;

    private static delay chosenDelay;
    private static autoMode chosenAuto;
    public static UiSmartDashboard uiSmartDashboard;
    private Timer elapsedTime;
    private boolean runAutoScheduler;
    public static boolean runningAutonomousMode; // used for other methods to know if robot is in automode
    public static SwerveDrivetrainMK3 practiceDrivetrain;
    public static Pigeon pigeon;
    public static Limelight limelightShooter;
    public static Limelight limelightCollector;
    public static OI oi;
    public static UI ui;
    public static SwerveDrivetrain swerveDrivetrain = null;
    public static DriveSwerve driveSwerve;
    private static TelemetryFileLogger telemetryLogger;
    public static EventFileLogger eventLogger;
    public static Shooter shooter;
    public static Wrist wrist;
    public static Indexer indexer;
    public static Collector collector;
    public static Elevator elevator;
    public static Climber climber;
    public static Diagnostics diagnostics;
    public static LEDSubsystem ledSubsystem;

    public static boolean isCompBot = true; //includes 2023 bot

    public static boolean manualControlMode;

    //https://github.com/mjansen4857/pathplanner/tree/main/examples/java/src/main/java/frc/robot

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        eventLogger = new EventFileLogger();
        eventLogger.init();

        telemetryLogger = new TelemetryFileLogger();
        addMetricsToLogger();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        elapsedTime = new Timer();

        limelightShooter = new Limelight("limelight-shooter");
        limelightCollector = new Limelight("limelight-collect");

        if (isCompBot) {
            pigeon = new Pigeon(RobotMap.PIGEON_ID);
            swerveDrivetrain = new SwerveDrivetrain(telemetryLogger);
            driveSwerve = new DriveSwerve();
            swerveDrivetrain.setDefaultCommand(driveSwerve);
            swerveDrivetrain.resetHeading();

        } else {
            pigeon = new Pigeon(RobotMap.PIGEON_PBOT_ID);
            practiceDrivetrain = new SwerveDrivetrainMK3();
            practiceDrivetrain.setDefaultCommand(new DrivePracticeSwerve());
            practiceDrivetrain.enableShuffleboardDebug("FRC2024");
        }

        shooter = new Shooter();

        try {
            wrist = new Wrist();
        } catch (Exception e) { throw new RuntimeException(e); }

        //        shooter.setDefaultCommand(new ShooterAngle());
        indexer = new Indexer();
        indexer.setDefaultCommand(new IndexerDefault());
        collector = new Collector();
        collector.setDefaultCommand(new CollectorDefault());
//        elevator = new Elevator();
        climber = new Climber();
        diagnostics = new Diagnostics();

        wrist.setDefaultCommand(new WristAnglePIDControl());

//        climber.setDefaultCommand(new ClimberPIDControl());

        ledSubsystem = new LEDSubsystem();
        ledSubsystem.setDefaultCommand(new LEDSubsystemDefault());

        robotContainer = new RobotContainer();

        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();

        oi = new OI();

        manualControlMode = false;
        runningAutonomousMode = false;
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

        if (diagnostics.getDriveMotorTempHot()) {
            swerveDrivetrain.stopModules();
            CommandScheduler.getInstance().disable();
        }
        if (diagnostics.getShooterMotorTempHot()) {
            shooter.stop();
        }
        if (diagnostics.getIndexerMotorTempHot()) {
            indexer.stopIndexerCoast();
        }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        runningAutonomousMode = true;

        chosenAuto = uiSmartDashboard.autoModeChooser.getSelected();
        chosenDelay = uiSmartDashboard.delayChooser.getSelected();

        autonomousCommand = robotContainer.getAutonomousCommand(chosenAuto);

        elapsedTime.reset();
        elapsedTime.start();
        runAutoScheduler = true;


        // TODO change - this is to offset the starting position of the auto
        swerveDrivetrain.resetOdometry(new Pose2d(new Translation2d(1.27, 5.54), pigeon.getRotation2d()));
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
        runningAutonomousMode = false;
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 40.0 && timeLeft >= 36.0) ||
                (timeLeft <= 25.0 && timeLeft >= 21.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));

//        shooter.updateShooterPID(); // used for tuning shooter PID using the dashboard
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

    public static void cancelAuto() {
        autonomousCommand.cancel();
    }

    public static boolean getManualControlMode() {
        return manualControlMode;
    }
}
