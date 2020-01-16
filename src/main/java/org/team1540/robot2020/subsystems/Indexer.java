package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax cord = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder cordEncoder = cord.getEncoder();

    private DigitalInput stagingSensor = new DigitalInput(0);
    private DigitalInput topSensor = new DigitalInput(0);

    private int balls = 0;
}
