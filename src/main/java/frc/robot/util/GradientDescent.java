package frc.robot.util;

public class GradientDescent {
    public static double[] minimize(double[] errorValues, double setpoint, double[] ts, double lambda, double Kp, double Ki, double Kd, double prevdKp, double prevdKi, double prevdKd) {
        double delKp = 0.0;
        double delKi = 0.0;
        double delKd = 0.0;

        double funcVal = 0.0;

        double summation = 0.0;
        double deriv = 0.0;

        double eti = 0.0;
        double etPrev = errorValues[0];
        double delT = 0.0;
        double delp1 = 0.0;

        for(int i = 1; i < errorValues.length; i++) {
            eti = errorValues[i];
            delT = ts[i] - ts[i - 1];

            summation += errorValues[i];
            deriv = ((eti - etPrev) / delT);
            funcVal = Kp * eti + Kd * deriv + Ki * summation;

            delp1 = setpoint - funcVal;

            delKp += delp1 * eti;
            delKi += delp1 * summation * delT;
            delKd += delp1 * deriv;
            
            etPrev = eti;
        }
        
        delKp *= lambda;
        delKi *= lambda;
        delKd *= lambda;
        double M = 0.8;
        delKp += prevdKp * M;
        delKi += prevdKi * M;
        delKd += prevdKd * M;
        
        return new double[] {delKp, delKi, delKd};
    }
}
