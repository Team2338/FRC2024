// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team.gif.lib.autoMode;
import team.gif.lib.delay;
import team.gif.lib.logging.EventFileLogger;
import team.gif.lib.logging.TelemetryFileLogger;
import team.gif.lib.shootParams;
import team.gif.robot.commands.climber.ClimberPIDHold;
import team.gif.robot.commands.collector.CollectorDefault;
import team.gif.robot.commands.drivetrain.DriveSwerve;
import team.gif.robot.commands.elevator.ElevatorPIDControl;
import team.gif.robot.commands.indexer.IndexerDefault;
import team.gif.robot.commands.led.LEDSubsystemDefault;
import team.gif.robot.commands.wrist.WristAnglePIDControl;
import team.gif.robot.subsystems.Climber;
import team.gif.robot.subsystems.Collector;
import team.gif.robot.subsystems.Diagnostics;
import team.gif.robot.subsystems.Elevator;
import team.gif.robot.subsystems.Indexer;
import team.gif.robot.subsystems.LEDSubsystem;
import team.gif.robot.subsystems.SensorMonitor;
import team.gif.robot.subsystems.Shooter;
import team.gif.robot.subsystems.Wrist;
import team.gif.robot.subsystems.SwerveDrivetrain;
import team.gif.robot.subsystems.SwerveDrivetrainMK3;
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
    public static SensorMonitor sensors;
    public static Shooter shooter;
    public static Wrist wrist;
    public static Indexer indexer;
    public static Collector collector;
    public static Elevator elevator;
    public static Climber climber;
    public static Diagnostics diagnostics;
    public static LEDSubsystem ledSubsystem;

    public static shootParams nextShot;

    public static boolean isCompBot = true; //includes 2023 bot

    public static boolean competitionMode = true;
    public static boolean fullDashboard = false;//!competitionMode;

    public static boolean manualControlMode;

    public static boolean killAutoAlign;

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

        limelightCollector = new Limelight("limelight-collect");
        limelightShooter = new Limelight("limelight-shooter");
        limelightShooter.setDistanceEstimatorParams(35,16.50,57,4);

        sensors = new SensorMonitor();

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

        nextShot = shootParams.WALL;

        shooter = new Shooter();

        try {
            wrist = new Wrist();
        } catch (Exception e) { throw new RuntimeException(e); }
        wrist.setDefaultCommand(new WristAnglePIDControl());

        indexer = new Indexer();
        indexer.setDefaultCommand(new IndexerDefault());
        collector = new Collector();
        collector.setDefaultCommand(new CollectorDefault());
        elevator = new Elevator();
        elevator.setDefaultCommand(new ElevatorPIDControl());
        climber = new Climber();
        climber.setTargetPosition(climber.getPosition());
        climber.setDefaultCommand(new ClimberPIDHold());
        diagnostics = new Diagnostics();
        ledSubsystem = new LEDSubsystem();
        ledSubsystem.setDefaultCommand(new LEDSubsystemDefault());

        robotContainer = new RobotContainer();

        ui = new UI();
        uiSmartDashboard = new UiSmartDashboard();

        oi = new OI();

        manualControlMode = false;
        runningAutonomousMode = false;

        //Increase the speed the sensors update at to 10ms, with offset of 5 ms from teleopPeriodic to avoid conflicts
        addPeriodic(() -> sensors.updateSensors(), 0.01, 0.005);
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

        // only stop the motors during practice
        if (!competitionMode) {
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

        wrist.enableAutoAngle();

        elapsedTime.reset();
        elapsedTime.start();
        runAutoScheduler = true;
        System.out.println("Autonomous Init");
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

        System.out.println(diagnostics.getRobotHasNote());
    }

    @Override
    public void teleopInit() {
        System.out.println("Teleop Init");

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        runningAutonomousMode = false;

        // Autos may start with a heading other than 0, but bot starts with 0 heading, Need to adjust pigeon.
        var alliance = DriverStation.getAlliance();
        if( alliance.isPresent() ) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                Robot.pigeon.resetPigeonPosition(Robot.pigeon.get360Heading() - robotContainer.getAutonomousInitialHeading());
            } else {
                Robot.pigeon.resetPigeonPosition(Robot.pigeon.get360Heading() + robotContainer.getAutonomousInitialHeading());
            }
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double timeLeft = DriverStation.getMatchTime();
        oi.setRumble((timeLeft <= 40.0 && timeLeft >= 36.0) ||
                (timeLeft <= 25.0 && timeLeft >= 21.0) ||
                (timeLeft <= 5.0 && timeLeft >= 3.0));

        telemetryLogger.run();
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
        telemetryLogger.addMetric("Target_RPM", () -> shooter.getTargetRPM());
        telemetryLogger.addMetric("RPM_Actual", () -> shooter.getShooterRPM());
        telemetryLogger.addMetric("Target_Wrist", () -> wrist.absoluteToDegrees(wrist.getTargetPosition()));
        telemetryLogger.addMetric("Wrist_Actual", () -> wrist.absoluteToDegrees(wrist.getPosition()));
        telemetryLogger.addMetric("Shooter_Sensor", () -> sensors.shooter());
        telemetryLogger.addMetric("Distance", () -> limelightShooter.getDistance());
        telemetryLogger.addMetric("YOffset", () -> limelightShooter.getYOffset());
        telemetryLogger.init();
    }

    public static void cancelAuto() {
        autonomousCommand.cancel();
    }

    public static boolean getManualControlMode() {
        return manualControlMode;
    }
}