// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class SensorMonitor extends SubsystemBase {
    private boolean collectorSensor = false;
    private boolean indexerSensor = false;
    private boolean shooterSensor = false;

    private DigitalInput collectorSensorPhysical;
    private DigitalInput indexerSensorPhysical;
    private DigitalInput shooterSensorPhysical;
    private Debouncer indexerSensorDebouncer;

    // The subsystem decreases the number of times the sensor is read every period to 1
    // Hopefully this will prevent overruns
    public SensorMonitor() {
        collectorSensorPhysical = new DigitalInput(RobotMap.SENSOR_COLLECTOR_PORT);
        indexerSensorPhysical = new DigitalInput(RobotMap.MIDDLE_SENSOR_PORT);
        shooterSensorPhysical = new DigitalInput(RobotMap.SHOOTER_SENSOR_PORT);

        indexerSensorDebouncer = new Debouncer(Constants.debounceDefault, Debouncer.DebounceType.kFalling);

    }

    public void updateSensors() {
        setCollectorSensorState(collectorSensorPhysical.get());
        setIndexerSensorState(indexerSensorDebouncer.calculate(indexerSensorPhysical.get()));
        setShooterSensorState(shooterSensorPhysical.get());
    }

    public boolean collector() {
        return collectorSensor;
    }

    public boolean indexer() {
        return indexerSensor;
    }

    public boolean shooter() {
        return shooterSensor;
    }

    private void setCollectorSensorState(boolean state) {
        collectorSensor = state;
    }

    private void setIndexerSensorState(boolean state) {
        indexerSensor = state;
    }

    private void setShooterSensorState(boolean state) {
        shooterSensor = state;
    }
}
