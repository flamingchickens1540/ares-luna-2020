/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/**
 * A command that does nothing but takes a specified amount of time to finish.  Useful for
 * CommandGroups.  Can also be subclassed to make a command with an internal timer.
 */
public class WaitSupplierCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    private final Supplier<Double> m_duration;

    /**
     * Creates a new WaitSupplierCommand.  This command will do nothing, and end after the specified duration.
     *
     * @param seconds the time to wait, in seconds
     */
    public WaitSupplierCommand(Supplier<Double> seconds) {
        m_duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_duration.get());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
