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

    public static CommandBase loop(Command commandToSchedule) {
        return new LoopCommand(commandToSchedule, () -> true);
    }

    public static CommandBase loop(Command commandToSchedule, int numberOfLoops) {
        return new LoopCommand(commandToSchedule, numberOfLoops);
    }

    public static CommandBase loop(Command commandToSchedule, Supplier<Boolean> loopWhileTrue) {
        return new LoopCommand(commandToSchedule, loopWhileTrue);
    }

    private final Command commandToSchedule;
    private int currentLoopIndex; // -1 means no command is scheduled
    private Supplier<Boolean> commandShouldBeRunning;
    private boolean commandIsRunning = false;

    private LoopCommand(Command commandToSchedule) {
        requireUngrouped(commandToSchedule);

        this.commandToSchedule = new SequentialCommandGroup(commandToSchedule); // workaround for CommandGroupBase.registerGroupedCommands being package-private

        m_requirements.addAll(this.commandToSchedule.getRequirements());
    }

    public LoopCommand(Command commandToSchedule, int numberOfLoops) {
        this(commandToSchedule);
        commandShouldBeRunning = () -> currentLoopIndex < numberOfLoops;
    }

    public LoopCommand(Command commandToSchedule, Supplier<Boolean> loopWhileTrue) {
        this(commandToSchedule);
        commandShouldBeRunning = loopWhileTrue;
    }

    @Override
    public void initialize() {
        currentLoopIndex = 0;
        if (commandShouldBeRunning.get()) {
            commandToSchedule.initialize();
            commandIsRunning = true;
        }
    }

    @Override
    public void execute() {
        commandToSchedule.execute();
        if (commandToSchedule.isFinished()) {
            commandToSchedule.end(false);
            currentLoopIndex++;
            commandIsRunning = commandShouldBeRunning.get();
            if (commandIsRunning) {
                commandToSchedule.initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && commandIsRunning) {
            commandToSchedule.end(true);
        }
        commandIsRunning = false;
    }

    public int getCurrentLoopIndex() {
        return currentLoopIndex;
    }

    @Override
    public boolean isFinished() {
        return commandIsRunning;
    }

    @Override
    public boolean runsWhenDisabled() {
        return commandToSchedule.runsWhenDisabled();
    }
}
