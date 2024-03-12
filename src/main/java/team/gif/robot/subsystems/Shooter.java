package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.lib.shootParams;

public class Shooter extends SubsystemBase {
//    public static CANSparkMax shooterNeo; //Leave for shooter Neo
    public static CANSparkFlex shooter;
    public static SparkPIDController pidShooter;

    private shootParams currentShot;

    public Shooter() {
//        shooterNeo = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless); // Leave for shooter Neo
        shooter = new CANSparkFlex(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        configShooter();
    }

    public void setVoltagePercent(double percent) {
//        shooterNeo.set(percent); // Leave for shooter Neo
        shooter.set(percent);
    }

    /**
     * Sets the shooter RPM
     *
     * @param rpm Desired velocity of shooter in RPM
     */
    public void setShooterRPM(double rpm) {
//        pidShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity); // Leave for shooter Neo
        pidShooter.setReference(rpm, CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Sets up the shooter RPM and revs the flywheel
     *
     * @param rpm Desired velocity of shooter in RPM
     */
    public void setupAndRev(double rpm) {
        setupNextShot();
        setShooterRPM(rpm);
    }


    public void setShooterRPMIdle() {
        setShooterRPM(Constants.Shooter.IDLE_RPM);
    }

    public double getShooterRPM() {
//        return shooterNeo.getEncoder().getVelocity(); // Leave for shooter Neo
        return shooter.getEncoder().getVelocity();
    }

    public boolean getShooterAtMinRPM() {
        return getShooterRPM() > Robot.nextShot.getMinimumRPM();
    }

    public void stop() {
        shooter.setVoltage(0);
    }

    public String getShooterRPM_Shuffleboard() {
        return String.format("%12.0f", getShooterRPM());
    }

    public void resetKI() {
        pidShooter.setIAccum(0.0);
        pidShooter.setIZone(1000);
    }

    /** used for tuning shooter PID on the dashboard
     *
     */
    public void updateShooterPID() {
        double shooterFF = SmartDashboard.getNumber("FF",0);
        double shooterKP = SmartDashboard.getNumber("kP",0);
        double shooterKI = SmartDashboard.getNumber("kI",0);

        System.out.println(shooterFF*1000 + "    " + shooterKP*1000 + "    " + shooterKI*1000);
        pidShooter.setFF(shooterFF);
        pidShooter.setP(shooterKP);
        pidShooter.setI(shooterKI);
    }

    public double getShooterMotorTemp() {
        return shooter.getMotorTemperature();
    }

    public boolean isShooterCool() {
        return !(getShooterMotorTemp() >= Constants.MotorTemps.SHOOTER_MOTOR_TEMP);
    }

    /**
     *  All the config setting for Shooter (controller, pid)
     */
    public void configShooter() {
//        shooterNeo.restoreFactoryDefaults(); // Leave for shooter Neo
//        shooterNeo.setInverted(true); // Leave for shooter Neo
//        shooterNeo.setIdleMode(CANSparkBase.IdleMode.kCoast); // Leave for shooter Neo

        shooter.restoreFactoryDefaults();
        shooter.setInverted(true);
        shooter.setIdleMode(CANSparkBase.IdleMode.kCoast);

//        pidShooter = shooterNeo.getPIDController(); // Leave for shooter Neo
        pidShooter = shooter.getPIDController();

        pidShooter.setFF(Robot.nextShot.getFF());
        pidShooter.setP(Robot.nextShot.getP());
        pidShooter.setI(Robot.nextShot.getI());
        pidShooter.setOutputRange(0,1);
        pidShooter.setIAccum(0.0);
        pidShooter.setIZone(1000);
    }

    /**
     * Sets up the parameters for the next shot the robot will take
     *
     * @param
     */
    public void setupNextShot() {
        // setting the gains is very expensive in time causing schedule loop overruns
        // only change if next shot is different from previous shot
        if (Robot.nextShot != currentShot) {
            pidShooter.setP(Robot.nextShot.getP());
            pidShooter.setFF(Robot.nextShot.getFF());
            pidShooter.setI(Robot.nextShot.getI());
            currentShot = Robot.nextShot;
        }
    }
}
