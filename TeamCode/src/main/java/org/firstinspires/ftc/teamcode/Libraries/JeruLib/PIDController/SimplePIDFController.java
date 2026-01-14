package org.firstinspires.ftc.teamcode.Libraries.JeruLib.PIDController;

public class SimplePIDFController extends SimplePIDController {
    private double m_kf;
    private double m_ks, m_kv, m_ka;
    public SimplePIDFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd);
        setFF(kf);
    }
    public SimplePIDFController(double kp, double ki, double kd, double ks, double kv, double ka) {
        super(kp, ki, kd);
        setFF(ks,kv,ka);
    }
    public void setFF(double kf) {m_kf = kf;}
    public void setFF(double ks, double kv, double ka) {
        setKs(ks);
        setKv(kv);
        setKa(ka);
    }
    public void setKs(double ks) {
        if (ks < 0.0) {
            throw new IllegalArgumentException("kv must be a non-negative number, got " + ks + "!");
        }
        m_ks = ks;
    }
    public void setKv(double kv) {
        if (kv < 0.0) {
            throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
        }
        m_kv = kv;
    }
    public void setKa(double ka) {
        if (ka < 0.0) {
            throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
        }
        m_ka = ka;
    }

    private double calculateFF(double velocity, double acceleration) {
        return m_ks * Math.signum(velocity) + m_kv * velocity + m_ka * acceleration;
    }
    private double calculateFF(double velocity) {
        return calculateFF(velocity,0);
    }


    public double calculate(double measurement, double setpoint, double velocity, double acceleration) {
        return calculateFF(velocity, acceleration) + super.calculate(measurement, setpoint);
    }
    public double calculate(double measurement, double velocity, double acceleration) {
        return calculate(measurement, 0, velocity, acceleration);
    }
    public double calculate(double measurement, double velocity) {
        return calculate(measurement, 0, velocity, 0);
    }
}
