package org.team1540.robot2020.utils;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import org.jetbrains.annotations.NotNull;

public class MotorConfigUtils {
    @NotNull
    public static TalonFXConfiguration get1540DefaultTalonFXConfiguration() {
        TalonFXConfiguration defaultTalonFXConfiguration = new TalonFXConfiguration();
        defaultTalonFXConfiguration.voltageCompSaturation = 12;
        defaultTalonFXConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
        defaultTalonFXConfiguration.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);
        defaultTalonFXConfiguration.openloopRamp = 0;
        return defaultTalonFXConfiguration;
    }
}
