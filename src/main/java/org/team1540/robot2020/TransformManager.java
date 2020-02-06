package org.team1540.robot2020;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2020.utils.LIDARLite;

public class TransformManager {
    private LIDARLite lidarLite;

    public TransformManager(LIDARLite lidarLite) {
        this.lidarLite = lidarLite;
    }

    public void initialize() {
        lidarLite.startMeasuring();
    }

    public void periodic() {
        double value = lidarLite.getDistance();
        SmartDashboard.putNumber("sensors/lidarDistance", value);
        System.out.println(value);
    }
}
