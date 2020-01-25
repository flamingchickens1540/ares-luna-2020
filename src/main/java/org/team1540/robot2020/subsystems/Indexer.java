package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax cord = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder cordEncoder = cord.getEncoder();

    private AnalogInput stagingSensor = new AnalogInput(0);
    private AnalogInput topSensor = new AnalogInput(0);

    private int balls = 0;

    public Indexer() {
        cord.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPercent(double speed) {
        cord.set(speed);
    }

    public int getStagingSensor() {
        return stagingSensor.getValue();
    }

    public int getTopSensor() {
        return topSensor.getValue();
    }

    public double getEncoderInches() {
        return cordEncoder.getPosition();
    }

    public void resetEncoder() {
        cordEncoder.setPosition(0);
    }

    public int getBalls() {
        return balls;
    }

    public void addBall() {
        balls++;
    }

    public void removeBall() {
        balls--;
    }
}
