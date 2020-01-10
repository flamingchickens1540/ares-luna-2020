package org.team1540.robot2020.shouldbeinrooster;

public class ConstantOptimizer {

    private double currentGuess;
    private double currentStepSize;
    private boolean nextStepPositive;

    private double lastGuessWeight = Double.POSITIVE_INFINITY;

    public ConstantOptimizer(double initialGuess, boolean startStepPositive, double initialStepSize) {
        currentGuess = initialGuess;
        nextStepPositive = startStepPositive;
        currentStepSize = initialStepSize;
    }

    public void setCurrentGuess(double currentGuess) {
        this.currentGuess = currentGuess;
    }

    public void setCurrentStepSize(double currentStepSize) {
        this.currentStepSize = currentStepSize;
    }

    public void setNextStepPositive(boolean nextStepPositive) {
        this.nextStepPositive = nextStepPositive;
    }

    public double getCurrentGuess() {
        return currentGuess;
    }

    public boolean isNextStepPositive() {
        return nextStepPositive;
    }

    public double getLastGuessWeight() {
        return lastGuessWeight;
    }

    public double computeNextGuess(double currentGuessWeight) {
        if (currentGuessWeight >= lastGuessWeight) {
            currentStepSize /= 2;
            nextStepPositive = !nextStepPositive;
        }

        currentGuess += (nextStepPositive ? 1 : -1) * currentStepSize;
        lastGuessWeight = currentGuessWeight;
        return getCurrentGuess();
    }
}

