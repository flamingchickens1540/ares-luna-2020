package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Avian {
    private String[] detections;

    public Avian() {
        refreshDetections();
    }

    public void refreshDetections() {
        this.detections = SmartDashboard.getStringArray("avian/detections", new String[]{});
    }

    public boolean getBoolean(String path) {
        return SmartDashboard.getBoolean(path, false);
    }

    public double getDouble(String path) {
        return SmartDashboard.getNumber(path, 0);
    }

    public boolean getBooleanExclusive(String path) {
        // If any other detection is true, return false

        // Left hand
        for (String detection : this.detections) {
            String detectionPath = "avian/left_" + detection;
            if (!path.equals(detectionPath)) {
                if (SmartDashboard.getBoolean(detectionPath, false)) {
                    return false;
                }
            }
        }

        // Right hand
        for (String detection : this.detections) {
            String detectionPath = "avian/right_" + detection;
            if (!path.equals(detectionPath)) {
                if (SmartDashboard.getBoolean(detectionPath, false)) {
                    return false;
                }
            }
        }

        return SmartDashboard.getBoolean(path, false);
    }
}
