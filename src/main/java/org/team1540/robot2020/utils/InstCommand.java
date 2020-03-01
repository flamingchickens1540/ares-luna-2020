package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

public class InstCommand extends CommandBase {
    private final Runnable toRun;
    private final boolean runWhenDisabled;

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given requirements.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems required by this command
     */
    public InstCommand(Runnable toRun, Subsystem... requirements) {
        this.toRun = requireNonNullParam(toRun, "toRun", "InstantCommand");

        addRequirements(requirements);

        runWhenDisabled = false;
    }

    public InstCommand(Runnable toRun, boolean runWhenDisabled, Subsystem... requirements) {
        this.toRun = requireNonNullParam(toRun, "toRun", "InstantCommand");

        addRequirements(requirements);

        this.runWhenDisabled = runWhenDisabled;
    }

    /**
     * Creates a new InstantCommand with a Runnable that does nothing.  Useful only as a no-arg
     * constructor to call implicitly from subclass constructors.
     */
    public InstCommand() {
        toRun = () -> {
        };

        runWhenDisabled = false;
    }

    public InstCommand(boolean runWhenDisabled) {
        toRun = () -> {
        };

        this.runWhenDisabled = runWhenDisabled;
    }

    @Override
    public void initialize() {
        toRun.run();
    }

    @Override
    public final boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }
}
