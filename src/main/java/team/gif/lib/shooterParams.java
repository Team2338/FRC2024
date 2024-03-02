package team.gif.lib;

import team.gif.robot.Constants;

public enum shooterParams {
    WALL(Constants.Wrist.SETPOINT_WALL_ABSOLUTE, Constants.Shooter.RPM_WALL, Constants.Shooter.kP_WALL, Constants.Shooter.kI_WALL, Constants.Shooter.FF_WALL),
    NEAR(Constants.Wrist.SETPOINT_NEAR_ABSOLUTE, Constants.Shooter.RPM_NEAR, Constants.Shooter.kP_NEAR, Constants.Shooter.kI_NEAR, Constants.Shooter.FF_NEAR),
    MID(Constants.Wrist.SETPOINT_MID_ABSOLUTE, Constants.Shooter.RPM_MID, Constants.Shooter.kP_MID, Constants.Shooter.kI_MID, Constants.Shooter.FF_MID),
    FAR(Constants.Wrist.SETPOINT_FAR_ABSOLUTE, Constants.Shooter.RPM_FAR, Constants.Shooter.kP_FAR, Constants.Shooter.kI_FAR, Constants.Shooter.FF_FAR);


    private double wristAngle;
    private double shooterRPM;
    private double kP;
    private double kI;
    private double FF;

    shooterParams(double wristAngle, double shooterRPM, double kP, double kI, double FF) {
        this.wristAngle = wristAngle;
        this.shooterRPM = shooterRPM;
        this.kP = kP;
        this.kI = kI;
        this.FF = FF;
    }

    public double getWristAngle() {
        return this.wristAngle;
    }
    public double getShooterRPM() {
        return this.shooterRPM;
    }
    public double getP() {
        return this.kP;
    }
    public double getI() {
        return this.kI;
    }
    public double getFF() {
        return this.FF;
    }

}
