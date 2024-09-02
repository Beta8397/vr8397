package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class QuinticSpline2D {

    private QuinticSpline xSpline, ySpline;
    private double length;

    public QuinticSpline2D( double x0, double y0, double x1, double y1, double tan0, double tan1){
        double alpha = 1;
        double beta = 1;
        double d = Math.hypot(y1 - y0, x1 - x0);
        double yDeriv0 = alpha * d * Math.sin(tan0);
        double xDeriv0 = alpha * d * Math.cos(tan0);
        double yDeriv1 = beta * d * Math.sin(tan1);
        double xDeriv1 = beta * d * Math.cos(tan1);
        xSpline = new QuinticSpline(x0, x1, xDeriv0, xDeriv1);
        ySpline = new QuinticSpline(y0, y1, yDeriv0, yDeriv1);
        length = getTotalLength();
    }


    public QuinticSpline2D( double x0, double y0, double x1, double y1, double tan0, double tan1,
                            double alpha, double beta){
        double d = Math.hypot(y1 - y0, x1 - x0);
        double yDeriv0 = alpha * d * Math.sin(tan0);
        double xDeriv0 = alpha * d * Math.cos(tan0);
        double yDeriv1 = beta * d * Math.sin(tan1);
        double xDeriv1 = beta * d * Math.cos(tan1);
        xSpline = new QuinticSpline(x0, x1, xDeriv0, xDeriv1);
        ySpline = new QuinticSpline(y0, y1, yDeriv0, yDeriv1);
        length = getTotalLength();
    }

    public VectorF p(double s){
        return new VectorF((float)xSpline.p(s), (float)ySpline.p(s));
    }

    public VectorF d1(double s){
        return new VectorF((float)xSpline.d1(s), (float)ySpline.d1(s));
    }

    public VectorF d2(double s){
        return new VectorF((float)xSpline.d2(s), (float)ySpline.d2(s));
    }

    private double getTotalLength(){
        double result = 0;
        for (int i=0; i<1000; i++){
            double s = 0.0005 + i/1000.0;
            VectorF d1 = d1(s);
            result += 0.001 * Math.sqrt(d1.get(0)*d1.get(0) + d1.get(1)*d1.get(1));
        }
        return result;
    }

    public double findClosestS(double x0, double y0, double startingS){
        int iter = 0;
        double s = startingS;
        while (iter < 100){
            double x = xSpline.p(s);
            double dx = xSpline.d1(s);
            double d2x = xSpline.d2(s);
            double y = ySpline.p(s);
            double dy = ySpline.d1(s);
            double d2y = ySpline.d2(s);
            double d1 = 2*( (x-x0)*dx + (y-y0)*dy );
            double d2 = 2*( (x-x0)*d2x + dx*dx + (y-y0)*d2y + dy*dy );
            if (Math.abs(d1) < 0.0001) break;
            s = s - d1/d2;
            iter++;
        }
        if (s < 0) s = 0;
        else if (s > 1) s = 1;
        return s;
    }

    public double getHeading(double s){
        return Math.atan2(ySpline.d1(s), xSpline.d1(s));
    }

    public double getHeadingRateOfChange(double s, double speed){
        double dx = xSpline.d1(s);
        double d2x = xSpline.d2(s);
        double dy = ySpline.d1(s);
        double d2y = ySpline.d2(s);
        return speed * (d2y*dx - dy*d2x) / Math.pow(dx*dx + dy*dy, 1.5);
    }

    public double getLength() { return length; }

}
