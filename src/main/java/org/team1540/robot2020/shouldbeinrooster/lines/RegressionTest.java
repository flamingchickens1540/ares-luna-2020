package org.team1540.robot2020.shouldbeinrooster.lines;

import java.util.Random;

public class RegressionTest {
    // https://stackoverflow.com/questions/17592139/trend-lines-regression-curve-fitting-java-library

    public static void main(String[] args) {
        PolyTrendLine t = new PolyTrendLine(5);
        Random rand = new Random();
        double[] x = new double[10 * 10];
        double[] err = new double[x.length];
        double[] y = new double[x.length];

        for (int i = 0; i < x.length; i++) {
            x[i] = 1000 * rand.nextDouble();
        }

        for (int i = 0; i < x.length; i++) {
            err[i] = 100 * rand.nextGaussian();
        }

        for (int i = 0; i < x.length; i++) {
            y[i] = x[i] * x[i] + err[i];
        } // quadratic model

        t.setValues(y, x);

        t.getCoef();



//        System.out.println(coef.getRowDimension());


//        System.out.println();
//
//        for (int i = 0; i < x.length; i++) {
//            System.out.println(x[i] + ", " + y[i]);
//        }

//        System.out.println("\n\n\n\n\n");

//
//        for (int i = 0; i < 1000; i++) {
//            System.out.println(i + ", " + t.predict(i));
//        }


//        System.out.println(); // when x=12, y should be... , eg 143.61380202745192
    }
}