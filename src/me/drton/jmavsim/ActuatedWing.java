package me.drton.jmavsim;

import java.util.List;
import java.util.Vector;

import javax.vecmath.Vector3d;
import javax.vecmath.Matrix3d;

import me.drton.jmavsim.Rxyz;

/**
 * Simple rotor model. Thrust and torque are proportional to control signal filtered with simple LPF (RC filter), to
 * simulate spin up/slow down.
 */





public class ActuatedWing {

    private boolean armed = false;
    private double tau = 1.0;
    private double fullThrust = 1.0;
    private double fullTorque = 1.0;
    private double w = 0.0;
    private long lastTime = -1;

    private Vector3d Force = new Vector3d();
    private Vector3d Torque = new Vector3d();
    private Vector3d wind = new Vector3d();

    public final double rho = 1.225;    // air density at sea level (kg/m³)

    // Environment
    private Vector3d position;
    private Vector3d velocity;
    private Vector3d acceleration;
    private Vector3d rotationRate;
    private Vector3d airSpeed;
    protected Matrix3d rotation = new Matrix3d();

    private List<Double> control = List.of(0.0, 0.0, 0.0, 0.0, 0.0); // [elevator, aileron, rudder, throttle]


    // Geometry
    public final double S_wing = 0.75;
    public final double b = 2.1;
    public final double c = 0.35714285714285715;
    public final double S_prop = 0.10178760197630929;

    // Propulsion
    public final double k_motor = 40;
    public final double k_T_P = 0;
    public final double k_Omega = 0;
    public final double C_prop = 1;

    // Lift Coefficients
    public final double C_L_alpha = 4.0203282440006793;
    public final double C_L_0 = 0.08673556671610734;
    public final double C_L_q = 3.87;
    public final double C_L_delta_e = 0.27807362017347131;

    // Drag Coefficients
    public final double C_D_0 = 0.01970001181915082;
    public final double C_D_alpha1 = 0.079091463157662967;
    public final double C_D_alpha2 = 1.0554699867680841;
    public final double C_D_beta1 = -0.0058429803454153884;
    public final double C_D_beta2 = 0.14781193079241584;
    public final double C_D_q = 0;
    public final double C_D_delta_e = 0.063347396781802318;

    // Moment Coefficients
    public final double C_m_alpha = -0.4629;
    public final double C_m_0 = 0.02275;
    public final double C_m_q = -1.3012370370370372;
    public final double C_m_delta_e = -0.2292;

    // Side force Coefficients
    public final double C_Y_0 = 0;
    public final double C_Y_beta = -0.22387215700254048;
    public final double C_Y_p = -0.13735505263157893;
    public final double C_Y_r = 0.083868768421052634;
    public final double C_Y_delta_a = 0.043276402502774876;
    public final double C_Y_delta_r = 0;

    // Roll moment Coefficients
    public final double C_l_0 = 0;
    public final double C_l_beta = -0.084896286396624165;
    public final double C_l_p = -0.40419799999999995;
    public final double C_l_r = 0.055520599999999996;
    public final double C_l_delta_a = 0.12018814125782745;
    public final double C_l_delta_r = 0;

    // Yaw moment Coefficients
    public final double C_n_0 = 0;
    public final double C_n_beta = 0.0283;
    public final double C_n_p = 0.0043655115789473682;
    public final double C_n_r = -0.072000000000000008;
    public final double C_n_delta_a = -0.00339;
    public final double C_n_delta_r = 0;

    double timeAccumulator = 0.0;

    // --------------------------------------------------------------
    public void update(long t, boolean paused) {


        if (paused) {
            return;
        }

        Force.set(0.0, 0.0, 0.0);
        Torque.set(0.0, 0.0, 0.0);

        if (lastTime >= 0) {

            
            double dt = (t - lastTime) / 1000.0;


            double p = rotationRate.x;
            double q = rotationRate.y;
            double r = rotationRate.z;


            double deg2rad = Math.PI / 180.0;

            // double elevator = (control.size() > 0 ? control.get(0) : 0.0) * 10.0 * deg2rad;
            // double leftAileron = control.size() > 1 ? control.get(1) : 0.0;
            // double rightAileron = control.size() > 2 ? control.get(2) : 0.0;
            // double aileron = (-rightAileron + leftAileron) * 0.5 * 10.0 * deg2rad;
            // double rudder = (control.size() > 3 ? control.get(3) : 0.0) * 10.0 * deg2rad;
            // double throttle = control.size() > 4 ? control.get(4) : 0.0;

            double throttle = 0.0;

            if ( this.armed ) {
                //throttle = 0.5 + (control.size() > 0 ? control.get(0) : 0.0)/2.0;
                throttle = (control.size() > 0 ? control.get(0) : 0.0);
            } 
            



            double leftElevon =  control.size() > 1 ? control.get(1) : 0.0;
            double rightElevon = control.size() > 2 ? control.get(2) : 0.0;


            double elevator = -0.5 * (leftElevon + rightElevon) * 40.0 * deg2rad;
            double aileron = 0.5 * (rightElevon - leftElevon) * 40.0 * deg2rad;
            double rudder = 0.0; //-(control.size() > 3 ? control.get(3) : 0.0) * 10.0 * deg2rad;


            // double elevator = -(control.size() > 1 ? control.get(1) : 0.0) * 30.0 * deg2rad;
            // double leftAileron = control.size() > 2 ? control.get(2) : 0.0;
            // double rightAileron = control.size() > 3 ? control.get(3) : 0.0;
            // double aileron = -0.5*(-rightAileron + leftAileron) * 30.0 * deg2rad ;
            // double rudder = 0.0; //-(control.size() > 3 ? control.get(4) : 0.0) * 10.0 * deg2rad;
            
   
            // Relative velocity
            
            Matrix3d rotationTranspose = new Matrix3d(); 
            rotationTranspose.set(rotation);
            rotationTranspose.transpose();

            Vector3d airSpeedBody = new Vector3d(this.airSpeed.x, this.airSpeed.y, this.airSpeed.z);
            rotationTranspose.transform(airSpeedBody);


            double u_r = airSpeedBody.x;
            double v_r = airSpeedBody.y;
            double w_r = airSpeedBody.z;

            // Airspeed, alpha, beta
            double Va = airSpeedBody.length();
            if (Va == 0) Va = 1e-5;

            double alpha = Math.atan2(w_r, u_r+0.00001);
            double beta = Math.asin(v_r / Va);


            // System.out.printf("throttle=%.2f, elev=%.2f, ail=%.2f, rudder=%.2f alpha=%.2f, beta=%.2f%n",
            //     throttle,
            //     elevator,
            //     aileron,
            //     rudder,
            //     alpha,
            //     beta
            // );


            timeAccumulator += dt;

            if (timeAccumulator >= 1.0) {
                System.out.printf(" alpha=%.2f, beta=%.2f, w_r=%.2f, u_r=%.2f, t=%.2f%n",
                    alpha/deg2rad,
                    beta/deg2rad,
                    w_r,
                    u_r,
                    throttle
                );

                timeAccumulator = 0.0;
            }

            // Lift
            double C_L_alpha = this.C_L_0 + this.C_L_alpha * alpha;
            double f_lift_s = 0.5 * this.rho * Va * Va * this.S_wing * (
                C_L_alpha + this.C_L_q * this.c / (2 * Va) * q + this.C_L_delta_e * elevator
            );

            // Drag
            double C_D_alpha = this.C_D_0 + this.C_D_alpha1 * alpha + this.C_D_alpha2 * alpha * alpha;
            double C_D_beta = this.C_D_beta1 * beta + this.C_D_beta2 * beta * beta;
            double f_drag_s = 0.5 * this.rho * Va * Va * this.S_wing * (
                C_D_alpha + C_D_beta + this.C_D_q * this.c / (2 * Va) * q + this.C_D_delta_e * elevator * elevator
            );

            // Pitching moment
            double m_a = this.C_m_0 + this.C_m_alpha * alpha;
            double m = 0.5 * this.rho * Va * Va * this.S_wing * this.c * (
                m_a + this.C_m_q * this.c / (2 * Va) * q + this.C_m_delta_e * elevator
            );

            // Lateral force
            double f_y = 0.5 * this.rho * Va * Va * this.S_wing * (
                this.C_Y_0 + this.C_Y_beta * beta + this.C_Y_p * this.b / (2 * Va) * p +
                this.C_Y_r * this.b / (2 * Va) * r + this.C_Y_delta_a * aileron + this.C_Y_delta_r * rudder
            );

            // Roll moment
            double l = 0.5 * this.rho * Va * Va * this.b * this.S_wing * (
                this.C_l_0 + this.C_l_beta * beta + this.C_l_p * this.b / (2 * Va) * p +
                this.C_l_r * this.b / (2 * Va) * r + this.C_l_delta_a * aileron + this.C_l_delta_r * rudder
            );

            // Yaw moment
            double n = 0.5 * this.rho * Va * Va * this.b * this.S_wing * (
                this.C_n_0 + this.C_n_beta * beta + this.C_n_p * this.b / (2 * Va) * p +
                this.C_n_r * this.b / (2 * Va) * r + this.C_n_delta_a * aileron + this.C_n_delta_r * rudder
            );

            // Aerodynamic forces in body frame
            Matrix3d R_alpha_beta = Rxyz.createRotationMatrix(0.0, alpha, beta);

            Vector3d F_aero = new Vector3d(-f_drag_s, f_y, -f_lift_s);
            R_alpha_beta.transform(F_aero);

            // Torques
            Vector3d T_aero = new Vector3d(l, m, n);

            // Propulsion
            double Vd = Va + throttle  * (this.k_motor - Va);
            Vector3d F_prop = new Vector3d(0.5 * this.rho * this.S_prop * this.C_prop * Vd * (Vd - Va), 0, 0);
            Vector3d T_prop = new Vector3d(-this.k_T_P * Math.pow(this.k_Omega * throttle, 2), 0, 0);

            // Total force and torque
            Force.add(F_prop);
            Force.add(F_aero);

            Torque.add(T_aero);
            Torque.add(T_prop);



        
        }
        lastTime = t;
    }

    /**
     * Set control signal
     * @param control control signal normalized to [0...1] for traditional or [-1...1] for reversable rotors
     */
    public void setControl(List<Double> control) {
        this.control = control;
    }

    public void setVehicleDynamics(Vector3d position, Vector3d velocity, Vector3d acceleration,
                                   Vector3d rotationRate, Matrix3d rotation) {
        this.position = position;   // in nav frame
        this.velocity = velocity;   // in nav frame
        this.acceleration = acceleration;
        this.rotationRate = rotationRate;
        this.rotation = rotation;  // Body FRD to NED navigation frame
    }

    public void setAirSpeed(Vector3d airSpeed) {
        this.airSpeed = airSpeed;   // in nav frame
    }

    public void setArmed(boolean armed) {
        this.armed = armed;   // in nav frame
    }

    /**
     * Set full thrust
     * @param fullThrust [N]
     */
    public void setFullThrust(double fullThrust) {
        this.fullThrust = fullThrust;
    }

    /**
     * Set torque at full thrust
     * @param fullTorque [N * m]
     */
    public void setFullTorque(double fullTorque) {
        this.fullTorque = fullTorque;
    }

    /**
     * Set time constant (spin-up time).
     * @param timeConstant [s]
     */
    public void setTimeConstant(double timeConstant) {
        this.tau = timeConstant;
    }

    /**
     * Get control signal
     */
    public double[] getControl() {
        double[] c = new double[5];
        for (int i = 0; i < 5; i++) {
            c[i] = i < control.size() ? control.get(i) : 0.0;
        }
        return c;
    }

    /**
     * Get current rotor thrust, [N]
     */
    public Vector3d getThrust() {
        return Force;
    }

    /**
     * Get current rotor torque [N * m]
     */
    public Vector3d getTorque() {
        return Torque;
    }

    /**
     * Get full thrust, [N]
     */
    public double getFullThrust() {
        return fullThrust;
    }

    /**
     * Get torque at full thrust, [N * m]
     */
    public double getFullTorque() {
        return fullTorque;
    }

    /**
     * Get time constant (spin-up time), [s].
     */
    public double getTimeConstant() {
        return tau;
    }
}
