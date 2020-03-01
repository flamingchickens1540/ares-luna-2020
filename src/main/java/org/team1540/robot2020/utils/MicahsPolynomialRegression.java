//package org.team1540.robot2020.utils;
//import Jama.Matrix;
//import Jama.QRDecomposition;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//
///**
// *  The {@code PolynomialRegression} class performs a polynomial regression
// *  on an set of <em>N</em> data points (<em>y<sub>i</sub></em>, <em>x<sub>i</sub></em>).
// *  That is, it fits a polynomial
// *  <em>y</em> = &beta;<sub>0</sub> +  &beta;<sub>1</sub> <em>x</em> +
// *  &beta;<sub>2</sub> <em>x</em><sup>2</sup> + ... +
// *  &beta;<sub><em>d</em></sub> <em>x</em><sup><em>d</em></sup>
// *  (where <em>y</em> is the response variable, <em>x</em> is the predictor variable,
// *  and the &beta;<sub><em>i</em></sub> are the regression coefficients)
// *  that minimizes the sum of squared residuals of the multiple regression model.
// *  It also computes associated the coefficient of determination <em>R</em><sup>2</sup>.
// *  <p>
// *  This implementation performs a QR-decomposition of the underlying
// *  Vandermonde matrix, so it is neither the fastest nor the most numerically
// *  stable way to perform the polynomial regression.
// *
// *  @author Robert Sedgewick
// *  @author Kevin Wayne
// */
//public class PolynomialRegression implements Comparable<PolynomialRegression> {
//    private String variableName;        // name of the predictor variable
//    private int degree;                 // degree of the polynomial regression
//    private Matrix beta;                // the polynomial regression coefficients
//    private double sse;                 // sum of squares due to error
//    private double sst;                 // total sum of squares
//    private double[] x;
//    private double[] y;
//
//
//    /**
//     * Performs a polynomial reggression on the data points {@code (y[i], x[i])}.
//     * Uses n as the name of the predictor variable.
//     *
//     * @param  x the values of the predictor variable
//     * @param  y the corresponding values of the response variable
//     * @param  degree the degree of the polynomial to fit
//     * @throws IllegalArgumentException if the lengths of the two arrays are not equal
//     */
//    public PolynomialRegression(double[] x, double[] y, int degree) {
//        this(x, y, degree, "x");
//    }
//
//    /**
//     * Performs a polynomial reggression on the data points {@code (y[i], x[i])}.
//     *
//     * @param  x the values of the predictor variable
//     * @param  y the corresponding values of the response variable
//     * @param  degree the degree of the polynomial to fit
//     * @param  variableName the name of the predictor variable
//     * @throws IllegalArgumentException if the lengths of the two arrays are not equal
//     */
//    public PolynomialRegression(double[] x, double[] y, int degree, String variableName) {
//        init(x,y,degree,variableName);
//    }
//
//    public PolynomialRegression(int degree) {
//        this(new double[]{}, new double[]{},degree);
//    }
//
//    private void init(double[] x, double[] y, int degree, String variableName) {
//        this.x = x;
//        this.y = y;
//        this.degree = degree;
//        this.variableName = variableName;
//
//        if(x.length>degree+1) {
//            int n = x.length;
//            QRDecomposition qr = null;
//            Matrix matrixX = null;
//
//            // in case Vandermonde matrix does not have full rank, reduce degree until it does
//            while (true) {
//
//                // build Vandermonde matrix
//                double[][] vandermonde = new double[n][this.degree + 1];
//                for (int i = 0; i < n; i++) {
//                    for (int j = 0; j <= this.degree; j++) {
//                        vandermonde[i][j] = Math.pow(x[i], j);
//                    }
//                }
//                matrixX = new Matrix(vandermonde);
//
//                // find least squares solution
//                qr = new QRDecomposition(matrixX);
//                if (qr.isFullRank()) break;
//
//                // decrease degree and try again
//                this.degree--;
//            }
//
//            // create matrix from vector
//            Matrix matrixY = new Matrix(y, n);
//
//            // linear regression coefficients
//            beta = qr.solve(matrixY);
//
//            // mean of y[] values
//            double sum = 0.0;
//            for (int i = 0; i < n; i++)
//                sum += y[i];
//            double mean = sum / n;
//
//            // total variation to be accounted for
//            for (int i = 0; i < n; i++) {
//                double dev = y[i] - mean;
//                sst += dev * dev;
//            }
//
//            // variation not accounted for
//            Matrix residuals = matrixX.times(beta).minus(matrixY);
//            sse = residuals.norm2() * residuals.norm2();
//        }
//    }
//
//    /**
//     * Returns the {@code j}th regression coefficient.
//     *
//     * @param  j the index
//     * @return the {@code j}th regression coefficient
//     */
//    public double beta(int j) {
//        testState();
//        // to make -0.0 print as 0.0
//        if (Math.abs(beta.get(j, 0)) < 1E-4) return 0.0;
//        return beta.get(j, 0);
//    }
//
//    public void add(double x, double y) {
//        double[] newX = new double[this.x.length+1];
//        double[] newY = new double[this.y.length+1];
//        for (int i = 0; i < this.x.length; i++) {
//            newX[i] = this.x[i];
//            newY[i] = this.y[i];
//        }
//        newX[newX.length-1] = x;
//        newY[newY.length-1] = y;
//        init(newX,newY,degree,variableName);
//    }
//
//    public void add(double[] x, double[] y) {
//        if(x.length!=y.length) throw(new IllegalArgumentException("both arrays must be of the same length"));
//        double[] newX = new double[this.x.length+x.length];
//        double[] newY = new double[this.y.length+x.length];
//        for (int i = 0; i < this.x.length; i++) {
//            newX[i] = this.x[i];
//            newY[i] = this.y[i];
//        }
//        for (int i = this.x.length; i < newX.length; i++) {
//            newX[i] = x[i-this.x.length];
//            newY[i] = y[i-this.y.length];
//        }
//        init(newX,newY,degree,variableName);
//    }
//
//    public void clear() {
//        init(new double[]{}, new double[]{},degree,variableName);
//    }
//
//    public void setDegree(int degree) {
//        if(degree<0) throw(new IllegalArgumentException("Degree cannot be negative"));
//        init(x,y,degree,variableName);
//    }
//
//    //Generates an array of coefficients from lowest n degree to highest
//    public double[] getCoefficients() {
//        testState();
//        double[] output = new double[degree+1];
//        for (int i = 0; i < degree+1; i++) {
//            output[i] = beta(i);
//        }
//        return output;
//    }
//
//    public String getLatexString() {
//        testState();
//        return toString();
//    }
//
//    /**
//     * Returns the degree of the polynomial to fit.
//     *
//     * @return the degree of the polynomial to fit
//     */
//    public int degree() {
//        return degree;
//    }
//
//    /**
//     * Returns the coefficient of determination <em>R</em><sup>2</sup>.
//     *
//     * @return the coefficient of determination <em>R</em><sup>2</sup>,
//     *         which is a real number between 0 and 1
//     */
//    public double R2() {
//        testState();
//        if (sst == 0.0) return 1.0;   // constant function
//        return 1.0 - sse/sst;
//    }
//
//    /**
//     * Returns the expected response {@code y} given the value of the predictor
//     *    variable {@code x}.
//     *
//     * @param  x the value of the predictor variable
//     * @return the expected response {@code y} given the value of the predictor
//     *         variable {@code x}
//     */
//    public double getDoubleStairStep(double x) {
//        testState();
//        // horner's method
//        double y = 0.0;
//        for (int j = degree; j >= 0; j--)
//            y = beta(j) + (x * y);
//        return y;
//    }
//
//    private void testState() {
//        if(x.length<=degree+1) throw(new IllegalStateException("Add more data points [degree:"+degree+", points:"+x.length+"]"));
//    }
//
//    /**
//     * Returns a string representation of the polynomial regression model.
//     *
//     * @return a string representation of the polynomial regression model,
//     *         including the best-fit polynomial and the coefficient of
//     *         determination <em>R</em><sup>2</sup>
//     */
//    @Override
//    public String toString() {
//        testState();
//        StringBuilder s = new StringBuilder();
//        int j = degree;
//
//        // ignoring leading zero coefficients
//        while (j >= 0 && Math.abs(beta(j)) < 1E-5)
//            j--;
//
//        // create remaining terms
//        while (j >= 0) {
//            if      (j == 0) s.append(String.format("%.2f ", beta(j)));
//            else if (j == 1) s.append(String.format("%.2f %s + ", beta(j), variableName));
//            else             s.append(String.format("%.2f %s^%d + ", beta(j), variableName, j));
//            j--;
//        }
//
//        // replace "+ -2n" with "- 2n"
//        return s.toString().replace("+ -", "- ");
//    }
//
//    /**
//     * Compare lexicographically.
//     */
//    @Override
//    public int compareTo(PolynomialRegression that) {
//        testState();
//        double EPSILON = 1E-5;
//        int maxDegree = Math.max(this.degree(), that.degree());
//        for (int j = maxDegree; j >= 0; j--) {
//            double term1 = 0.0;
//            double term2 = 0.0;
//            if (this.degree() >= j) term1 = this.beta(j);
//            if (that.degree() >= j) term2 = that.beta(j);
//            if (Math.abs(term1) < EPSILON) term1 = 0.0;
//            if (Math.abs(term2) < EPSILON) term2 = 0.0;
//            if      (term1 < term2) return -1;
//            else if (term1 > term2) return +1;
//        }
//        return 0;
//    }
//
//    /**
//     * Unit tests the {@code PolynomialRegression} data type.
//     *
//     * @param args the command-line arguments
//     */
//}