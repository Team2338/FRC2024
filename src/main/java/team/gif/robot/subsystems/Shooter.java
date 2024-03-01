package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;
import team.gif.lib.shooterParams;

public class Shooter extends SubsystemBase {
//    public static CANSparkMax shooterNeo; //Leave for shooter Neo
    public static CANSparkFlex shooter;
    public static SparkPIDController pidShooter;

    private double shooterFF;
    private double shooterKP;
    private double shooterKI;

    private shooterParams shooterParams;


    public Shooter() {
//        shooterNeo = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless); // Leave for shooter Neo
        shooter = new CANSparkFlex(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterParams = team.gif.lib.shooterParams.NEAR;
        configShooter();
    }

    public void setVoltagePercent(double percent) {
//        shooterNeo.set(percent); // Leave for shooter Neo
        shooter.set(percent);
    }

    public void setShooterRPM(double rpm) {
//        pidShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity); // Leave for shooter Neo
        pidShooter.setReference(rpm, CANSparkFlex.ControlType.kVelocity);
    }

    public double getShooterRPM() {
//        return shooterNeo.getEncoder().getVelocity(); // Leave for shooter Neo
        return shooter.getEncoder().getVelocity();
    }

    public void stop() {
        shooter.set(0);
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
        shooterFF = SmartDashboard.getNumber("FF",0);
        shooterKP = SmartDashboard.getNumber("kP",0);
        shooterKI = SmartDashboard.getNumber("kI",0);

        System.out.println(shooterFF*1000 + "    " + shooterKP*1000 + "    " + shooterKI*1000);
        pidShooter.setFF(shooterFF);
        pidShooter.setP(shooterKP);
        pidShooter.setI(shooterKI);
    }

    public double getShooterMotorTemp() {
        return shooter.getMotorTemperature();
    }

    public boolean isShooterCool() {
        if (getShooterMotorTemp() >= Constants.MotorTemps.SHOOTER_MOTOR_TEMP) {
            return false;
        }
        return true;
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

        pidShooter.setP(shooterParams.getP());
        pidShooter.setFF(shooterParams.getFF());
        pidShooter.setI(shooterParams.getI());
        pidShooter.setOutputRange(0,1);
        pidShooter.setIAccum(0.0);
        pidShooter.setIZone(1000);
    }


    public void setShooterParams(shooterParams shooterParams) {
        this.shooterParams = shooterParams;

        pidShooter.setP(shooterParams.getP());
        pidShooter.setFF(shooterParams.getFF());
        pidShooter.setI(shooterParams.getI());
    }

    public shooterParams getShooterParams() {
        return this.shooterParams;
    }
}
