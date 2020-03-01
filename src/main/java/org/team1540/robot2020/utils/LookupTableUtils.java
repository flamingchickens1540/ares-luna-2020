package org.team1540.robot2020.utils;

public class LookupTableUtils {
    private static double line(double x1, double y1, double x2, double y2, double x) {
        return ((y2 - y1) / (x2 - x1)) * (x - x1) + y1;
    }

    /**
     * @param input x-value to calculate
     * @param xarr  sorted (smallest to largest) array of x-values
     * @param yarr  sorted (smallest to largest) array of y-values
     * @return the best y-value for the lookup table
     * TODO simplify this logic
     */
    public static double getDoubleLookupTable(double input, double[] xarr, double[] yarr) {
        if (input < xarr[0]) { // If distance less than smallest recorded distance
            double lowerX = xarr[0];
            double lowerY = yarr[0];

            double upperX = xarr[1];
            double upperY = yarr[1];

            return line(lowerX, lowerY, upperX, upperY, input);
        } else if (input > xarr[xarr.length - 1]) { // If distance greater than largest recorded value
            double lowerX = xarr[xarr.length - 2];
            double lowerY = yarr[yarr.length - 2];

            double upperX = xarr[xarr.length - 1];
            double upperY = yarr[yarr.length - 1];

            return line(lowerX, lowerY, upperX, upperY, input);
        } else { // If distance somewhere in the middle
            for (int i = 0; i < xarr.length - 1; i++) {
                double lowerX = xarr[i];
                double lowerY = yarr[i];

                double upperX = xarr[i + 1];
                double upperY = yarr[i + 1];

                if (lowerX < input && input < upperX) { // HOOD[i-1] < distance < HOOD[i+1]
                    return line(lowerX, lowerY, upperX, upperY, input);
                }
            }
        }
        return 0;
    }

    public static double getDoubleStairStep(double input, double[] xarr, double[] yarr) {
        for (int i = 0; i < xarr.length; i++) {
            if (xarr[i] > input) {
                if (i == 0) return yarr[i];
                double avg = (xarr[i - 1] + xarr[i]) / 2;
                if (input < avg) return yarr[i - 1];
                else return yarr[i];
            }
        }
        return yarr[yarr.length - 1];
    }
}
