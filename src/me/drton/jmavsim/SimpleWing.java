package me.drton.jmavsim;
import me.drton.jmavsim.Rxyz;

import javax.vecmath.Vector3d;
import javax.vecmath.Matrix3d;


/**
 * Quasi-steady fixed-wing aerodynamic model.
 *
 * - Polynomial CL/CD/CM valid in limited alpha range (degrees)
 * - Smooth transition to sphere (bluff-body) model for high alpha
 * - Numerically stable for all incidence angles
 */
public class SimpleWing {

    // ------------------------------------------------
    // Geometry & environment
    // ------------------------------------------------
    private double rho = 1.225;      // air density [kg/m^3]
    private double S   = 0.22;       // wing reference area [m^2]
    private double c   = 0.16;       // mean aerodynamic chord [m]

    // ------------------------------------------------
    // Polynomial coefficients (alpha in DEGREES)
    // CL = a3 + a2*a + a1*a^2 + a0*a^3
    // ------------------------------------------------
    private final double[] CL = new double[4];
    private final double[] CD = new double[4];
    private final double[] CM = new double[4];

    // ------------------------------------------------
    // Valid fit range and blending limits [deg]
    // ------------------------------------------------
    private static final double ALPHA_FIT_MIN = -14.0;
    private static final double ALPHA_FIT_MAX =  18.5;
    private static final double ALPHA_SPHERE  =  30.0;

    // Sphere model
    private static final double CD_SPHERE = 1.2;

    // ------------------------------------------------
    // State
    // ------------------------------------------------
    private  Vector3d airSpeed = new Vector3d();
    private  Vector3d forceBody = new Vector3d();
    private  Vector3d torqueBody = new Vector3d();
    private Vector3d rotationRate;
    protected Matrix3d rotation = new Matrix3d();

    // ------------------------------------------------
    // Update
    // ------------------------------------------------
    public void update(long t, boolean paused) {
        if (paused) return;
        computeForces();
    }

    // ------------------------------------------------
    // Aerodynamics
    // ------------------------------------------------
    private void computeForces() {

  

        double rp = rotationRate.x;
        double rq = rotationRate.y;
        double rr = rotationRate.z;


        Matrix3d rotationTranspose = new Matrix3d(); 
        rotationTranspose.set(rotation);
        rotationTranspose.transpose();

        Vector3d airSpeedBody = new Vector3d(this.airSpeed.x, this.airSpeed.y, this.airSpeed.z);
        rotationTranspose.transform(airSpeedBody);


        double u_r = airSpeedBody.x;
        double v_r = airSpeedBody.y;
        double w_r = airSpeedBody.z;

        // Airspeed, alpha, beta
        double V = airSpeedBody.length();
        if (V == 0) V = 1e-5;


        double alphaRad = Math.atan2(w_r, u_r);
        double betaRad = Math.asin(v_r / V);

        double alphaDeg = Math.toDegrees(alphaRad);

        if (Double.isNaN(alphaRad)) {
            throw new IllegalStateException(
                "Alpha is NaN! airSpeed=" + airSpeed
            );
        }


        double q = 0.5 * rho * V * V;

        // --- Polynomial coefficients (protected) ---
        double clPoly = poly(CL, clamp(alphaDeg, ALPHA_FIT_MIN, ALPHA_FIT_MAX));
        double cdPoly = poly(CD, clamp(alphaDeg, ALPHA_FIT_MIN, ALPHA_FIT_MAX));
        double cmPoly = poly(CM, clamp(alphaDeg, ALPHA_FIT_MIN, ALPHA_FIT_MAX));

        // --- Sphere model ---
        double clSphere = 0.0;
        double cdSphere = CD_SPHERE;

        // --- Blend factor ---
        double s = blend(Math.abs(alphaDeg));


        System.out.printf(" alpha=%.2f, blend=%.2f %n",
            alphaDeg,
            s
        );


        double cl = lerp(clPoly, clSphere, s);
        double cd = lerp(cdPoly, cdSphere, s);
        double cm = (1.0 - s) * cmPoly;

        // --- Forces ---
        double L = q * S * cl;
        double D = q * S * cd;
        double M = 0.0*q * S * c * cm;

        double ca = Math.cos(alphaRad);
        double sa2 = Math.sin(alphaRad);
        
        // Aerodynamic forces in body frame
        // Wind axes → body axes
        forceBody.set(
            -D * ca + L * sa2,
             0.0,
            -D * sa2 - L * ca
        );



        // Sphere moments (very small, mostly damping)
        Vector3d T_sphere = new Vector3d(
            -0.001 * rp,
            -0.001 * rq,
            -0.01 * rr
        );



        // Pitching moment (body Y-axis)
        torqueBody.set(T_sphere.x, T_sphere.y -M, T_sphere.z);
    }

    // ------------------------------------------------
    // Helpers
    // ------------------------------------------------
    private double poly(double[] p, double a) {
        return p[3]
             + p[2] * a
             + p[1] * a * a
             + p[0] * a * a * a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double lerp(double a, double b, double s) {
        return a * (1.0 - s) + b * s;
    }

    private double blend(double absAlphaDeg) {
        if (absAlphaDeg <= ALPHA_FIT_MAX) return 0.0;
        if (absAlphaDeg >= ALPHA_SPHERE)  return 1.0;
        return (absAlphaDeg - ALPHA_FIT_MAX) /
               (ALPHA_SPHERE  - ALPHA_FIT_MAX);
    }

    // ------------------------------------------------
    // Setters
    // ------------------------------------------------
    public void setAirSpeed(Vector3d v) {
        this.airSpeed = v;   // in nav frame
    }

    public void setWingArea(double S) {
        this.S = S;
    }

    public void setChord(double c) {
        this.c = c;
    }

    public void setAirDensity(double rho) {
        this.rho = rho;
    }

    public void setCL(double[] cl) {
        System.arraycopy(cl, 0, CL, 0, 4);
    }

    public void setCD(double[] cd) {
        System.arraycopy(cd, 0, CD, 0, 4);
    }

    public void setCM(double[] cm) {
        System.arraycopy(cm, 0, CM, 0, 4);
    }


    public void setVehicleDynamics(Vector3d rotationRate, Matrix3d rotation) {
        this.rotationRate = rotationRate;
        this.rotation = rotation;  // Body FRD to NED navigation frame
    }

    // ------------------------------------------------
    // Outputs
    // ------------------------------------------------
    public Vector3d getForce() {
        return new Vector3d(forceBody);
    }

    public Vector3d getTorque() {
        return new Vector3d(torqueBody);
    }
}