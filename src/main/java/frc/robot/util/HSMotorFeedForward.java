package frc.robot.util;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class HSMotorFeedForward extends SimpleMotorFeedforward
{
    public HSMotorFeedForward(double ks, double kv, double ka) {
        super(ks, kv, ka);
    }

    private double prevVelocity = 0;
    private double prevTime = 0;

    @Override
    public double calculate(double vel){
        double acceleration = (vel-prevVelocity)/(System.currentTimeMillis() - prevTime);
        prevVelocity = vel;
        prevTime = System.currentTimeMillis();
        return calculate(vel, acceleration);
    }
}