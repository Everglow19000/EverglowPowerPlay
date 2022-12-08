package org.firstinspires.ftc.teamcode.utils.PathTypes;

public class CubicPolynomial {

    double a, b, c ,d; //In the form ax^3+bx^2+cx+d

    public CubicPolynomial(double a, double b, double c, double d){

        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;

    }

    public double getYatPoint(double x){
        return a*Math.pow(x,3) + b*Math.pow(x,2) + c*x + d;
    }
}
