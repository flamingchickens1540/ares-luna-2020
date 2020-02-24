/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

public class LoopCommand extends CommandBase {
    private final Command commandToSchedule;
    private Supplier<Integer> numberOfLoopsSupplier;
    private int currentLoopIndex = -1; // -1 means no command is scheduled
    private Integer numberOfLoops; // -1 means loop forever


    public LoopCommand(Command commandToSchedule) {
        this(commandToSchedule, () -> -1);
    }

    public LoopCommand(Command commandToSchedule, Supplier<Integer> numberOfLoops) {
        requireUngrouped(commandToSchedule);
        this.numberOfLoopsSupplier = numberOfLoops;

        this.commandToSchedule = new SequentialCommandGroup(commandToSchedule); // workaround for CommandGroupBase.registerGroupedCommands being package-private

        m_requirements.addAll(this.commandToSchedule.getRequirements());
    }

    @Override
    public void initialize() {
        numberOfLoops = numberOfLoopsSupplier.get();
        currentLoopIndex = 0;
        if (numberOfLoops != 0)
            commandToSchedule.initialize();
    }

    @Override
    public void execute() {
        commandToSchedule.execute();
        if (commandToSchedule.isFinished()) {
            commandToSchedule.end(false);
            currentLoopIndex++;
            if (numberOfLoops < 0 || currentLoopIndex < numberOfLoops) {
                commandToSchedule.initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted // might need to interrupt command
                && currentLoopIndex > -1 // command has not been ended already
                && currentLoopIndex < numberOfLoops // there was a command that should have been running
        ) {
            commandToSchedule.end(true); // interrupt the command
        }
        currentLoopIndex = -1; // no command is scheduled anymore
    }

    @Override
    public boolean isFinished() {
        return currentLoopIndex == numberOfLoops;
    }

    @Override
    public boolean runsWhenDisabled() {
        return commandToSchedule.runsWhenDisabled();
    }
}
