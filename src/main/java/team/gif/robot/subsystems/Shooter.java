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
    public static CANSparkFlex shooterMotor;
    public static SparkPIDController pidShooter;

    // Shooter use only. Purpose is to avoid repeated calls with the same
    // parameters to the motor controller, reducing CAN utilization
    private shootParams currentShot;

    /**
     * Used with PID to set the desired RPM of the shooter
     */
    private double targetRPM;

    public Shooter() {
//        shooterNeo = new CANSparkMax(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless); // Leave for shooter Neo
        shooterMotor = new CANSparkFlex(RobotMap.SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless);
        configShooter();
        Robot.limelightShooter.setPipeline(0);
        targetRPM = 0;
    }

    public void setVoltagePercent(double percent) {
//        shooterNeo.set(percent); // Leave for shooter Neo
        shooterMotor.set(percent);
    }

    /**
     * Sets the shooter velocity (RPM) (i.e. runs the shooter)
     *
     * @param rpm Desired velocity of shooter in RPM
     */
    public void setShooterRPM(double rpm) {
//        pidShooter.setReference(rpm, CANSparkBase.ControlType.kVelocity); // Leave for shooter Neo
        targetRPM = rpm;
        pidShooter.setReference(rpm, CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Gets the shooter target velocity (RPM)
     *
     * @return  rpm Target velocity of shooter in RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Loads the "next shot" or auto shooter PID values into the shooter motor
     * controllers (if different from last call) <br>
     * Sets the target shooter RPM from next shot or auto shooter <br>
     * Revs the flywheel <br>
     * Does not move wrist <br>
     * <br>
     *
     */
    public void configMotorControllerAndRev() {
        setupNextShot();
        if (Robot.wrist.isAutoAngleEnabled()) {
            setShooterRPM(Robot.autoShooterRPM);
        } else {
            setShooterRPM(Robot.nextShot.getShooterRPM());
        }
    }


    public void setShooterRPMIdle() {
        setShooterRPM(Constants.Shooter.IDLE_RPM);
    }

    /**
     * Gets the shooter current velocity (RPM)
     *
     * @return velocity of shooter in RPM
     */
    public double getShooterRPM() {
//        return shooterNeo.getEncoder().getVelocity(); // Leave for shooter Neo
        return shooterMotor.getEncoder().getVelocity();
    }

    public boolean getShooterAtMinRPM() {
        return getShooterRPM() > Robot.nextShot.getMinimumRPM();
    }

    /**
     * Stops the shooter motor by setting voltage to 0 <br>
     * Coasts the shooter down to 0 velocity rather than PID hard stop
     *
     */
    public void stop() {
        shooterMotor.setVoltage(0);
    }

    public String getShooterRPM_Shuffleboard() {
        return String.format("%12.0f", getShooterRPM());
    }

    public void resetKI() {
        if (pidShooter.getI() != 0 ) {
            pidShooter.setIAccum(0.0);
            pidShooter.setIZone(1000);
        }
    }

    /**
     * used for tuning shooter PID on the dashboard
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
        return shooterMotor.getMotorTemperature();
    }

    public boolean isShooterCool() {
        return getShooterMotorTemp() < Constants.MotorTemps.SHOOTER_MOTOR_TEMP;
    }

    /**
     *  All the config setting for Shooter (controller, pid)
     */
    public void configShooter() {
//        shooterNeo.restoreFactoryDefaults(); // Leave for shooter Neo
//        shooterNeo.setInverted(true); // Leave for shooter Neo
//        shooterNeo.setIdleMode(CANSparkBase.IdleMode.kCoast); // Leave for shooter Neo

        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setInverted(true);
        shooterMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterMotor.enableVoltageCompensation(12);

        shooterMotor.burnFlash();

//        pidShooter = shooterNeo.getPIDController(); // Leave for shooter Neo
        pidShooter = shooterMotor.getPIDController();

        pidShooter.setFF(Robot.nextShot.getFF());
        pidShooter.setP(Robot.nextShot.getP());
        pidShooter.setI(Robot.nextShot.getI());
        pidShooter.setOutputRange(0,1);
        pidShooter.setIAccum(0.0);
        pidShooter.setIZone(1000);
    }

    /**
     * Configures the motor controller with the PID parameters for the
     * next shot (taken from nextShot or autoShooter). Only loads if shot is different
     * from last shot.
     *
     */
    public void setupNextShot() {
        // setting the gains is very expensive in time causing schedule loop overruns
        // only change if next shot is different from previous shot
        if (Robot.nextShot == shootParams.AUTO && Robot.autoParamsDirtyFlag) {
            pidShooter.setFF(Robot.autoShooterFF);
            pidShooter.setP(Robot.autoShooterkP);
            pidShooter.setI(Robot.autoShooterkI);

            currentShot = shootParams.AUTO;
        } else if (Robot.nextShot != currentShot) {
            pidShooter.setFF(Robot.nextShot.getFF());
            pidShooter.setP(Robot.nextShot.getP());
            pidShooter.setI(Robot.nextShot.getI());

            // Local use only. Only purpose is to store the shot to compare next to current to
            // know if new PID values need to be sent to motor controller
            currentShot = Robot.nextShot;
        }
    }

    public void setPipeline0() {
        Robot.limelightShooter.setPipeline(0);
    }

    public void setPipeline1() {
        Robot.limelightShooter.setPipeline(1);
    }

    public void setPipeline2() {
        Robot.limelightShooter.setPipeline(2);
    }
}
