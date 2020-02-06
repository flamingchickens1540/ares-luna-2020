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
        SmartDashboard.putNumber("LIDAR Distance", value); // 2.54 to convert to inches and +6 is the tuning offset
        System.out.println(value);
    }
}
