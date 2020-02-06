package org.team1540.robot2020.utils;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import org.jetbrains.annotations.NotNull;

public class MotorConfigUtils {
    @NotNull
    public static TalonFXConfiguration get1540DefaultTalonFXConfiguration() {
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        defaultConfig.voltageCompSaturation = 12;
        defaultConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);
        defaultConfig.openloopRamp = 0;
        return defaultConfig;
    }

    public static void setDefaultNEOConfiguration(CANSparkMax neo) {
        neo.restoreFactoryDefaults();
        neo.setSmartCurrentLimit(20);
    }
}
