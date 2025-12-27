package me.drton.jmavsim;

public class Servo2D {

    private static final double MAX_ANGLE = Math.toRadians(15.0);

    private static final double DEV_PITCH = Math.toRadians(0.0);
    private static final double DEV_YAW = Math.toRadians(0.0);

    private double tau = 0.1;        // [s] servo time constant
    private double pitch = 0.0;      // actual pitch angle [rad]
    private double yaw   = 0.0;      // actual yaw angle [rad]
    private double cmdPitch = 0.0;   // commanded pitch [rad]
    private double cmdYaw   = 0.0;   // commanded yaw [rad]

    private long lastTime = -1;

    public void update(long t, boolean paused) {
        if (paused) {
            return;
        }

        if (lastTime >= 0) {
            double dt = (t - lastTime) / 1000.0;
            double a = 1.0 - Math.exp(-dt / tau);

            pitch += (cmdPitch - pitch) * a;
            yaw   += (cmdYaw   - yaw)   * a;
        }
        lastTime = t;
    }

    /**
     * Set normalized servo command
     * @param pitch [-1..1] → ±15 deg
     * @param yaw   [-1..1] → ±15 deg
     */
    public void setCommand(double pitch, double yaw) {
        pitch = Math.max(-1.0, Math.min(1.0, pitch));
        yaw   = Math.max(-1.0, Math.min(1.0, yaw));

        this.cmdPitch = pitch * MAX_ANGLE;
        this.cmdYaw   = yaw   * MAX_ANGLE;
    }

    public void setTimeConstant(double tau) {
        this.tau = tau;
    }

    public double getPitch() {
        return pitch + DEV_PITCH;
    }

    public double getYaw() {
        return yaw + DEV_YAW;
    }
}