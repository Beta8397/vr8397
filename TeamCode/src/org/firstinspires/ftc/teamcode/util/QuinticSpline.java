package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.ParametricFunction;

public class QuinticSpline implements ParametricFunction {

    private double val0, deriv0, val1, deriv1;
    private double a3, a4, a5;

    public QuinticSpline(double val0, double val1, double deriv0, double deriv1){
        this.val0 = val0;
        this.val1 = val1;
        this.deriv0 = deriv0;
        this.deriv1 = deriv1;
        a3 = 10* val1 - 10*val0 - 6* deriv0 - 4* deriv1;
        a4 = 8* deriv0 + 7* deriv1 - 15* val1 + 15*val0;
        a5 = 3 * (2* val1 - 2*val0 - deriv0 - deriv1);
    }

    @Override
    public double p(double s) {
        return val0 + s * (deriv0 + s*s*(a3 + s*(a4 + s*a5)));
    }

    @Override
    public double d1(double s) {
        return deriv0 + s*s*(3*a3 + s*(4*a4 + s*5*a5));
    }

    @Override
    public double d2(double s) {
        return s*(6*a3 + s*(12*a4 + s*20*a5));
    }
}
