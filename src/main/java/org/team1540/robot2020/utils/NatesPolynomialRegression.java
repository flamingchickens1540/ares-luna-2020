package org.team1540.robot2020.utils;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;

import java.util.ArrayList;

public class NatesPolynomialRegression {

    final int degree;
    double[] coeffs;

    ArrayList<Double> x = new ArrayList<>();
    ArrayList<Double> y = new ArrayList<>();

    public NatesPolynomialRegression(int degree) {
        if (degree < 0) throw new IllegalArgumentException("The degree of the polynomial must not be negative.");
        this.degree = degree;
        this.coeffs = new double[degree];
    }

    public double[] getX() {
        double[] xarr = new double[x.size()];
        for (int kX = 0; kX < x.size(); kX++) {
            xarr[kX] = x.get(kX);
        }
        return xarr;
    }

    public double[] getY() {
        double[] yarr = new double[y.size()];
        for (int kY = 0; kY < y.size(); kY++) {
            yarr[kY] = y.get(kY);
        }
        return yarr;
    }

    /**
     * Add a value to the regression arrays
     *
     * @param x X value
     * @param y Y value
     */
    public void add(double x, double y) {
        this.x.add(x);
        this.y.add(y);
    }

    /**
     *
     * @param x
     * @return Array
     */
    public double[] xVector(double x) { // {1, x, x*x, x*x*x, ...}
        double[] poly = new double[degree + 1];
        double xi = 1;
        for (int i = 0; i <= degree; i++) {
            poly[i] = xi;
            xi *= x;
        }
        return poly;
    }

    public void run() {
        if (x.size() != y.size()) {
            throw new IllegalArgumentException("Length of x and y must be equal. " + y.size() + " != " + x.size());
        }

        double[][] xData = new double[x.size()][];
        for (int i = 0; i < x.size(); i++) {
            xData[i] = xVector(x.get(i));
        }

        OLSMultipleLinearRegression ols = new OLSMultipleLinearRegression();
        ols.setNoIntercept(true);

        double[] yarr = new double[y.size()];
        for (int kY = 0; kY < x.size(); kY++) {
            yarr[kY] = x.get(kY);
        }

        ols.newSampleData(yarr, xData);
        coeffs = ols.estimateRegressionParameters();
    }

    public double[] get() {
        return coeffs;
    }
}
