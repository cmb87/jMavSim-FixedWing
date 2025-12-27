package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.Servo2D;
import me.drton.jmavsim.ActuatedWing;
import me.drton.jmavsim.World;
import me.drton.jmavsim.ForceTorque;

import java.util.Arrays;

import javax.vecmath.Vector3d;

/**
 * Abstract multicopter class. Does all necessary calculations for multirotor with any placement of rotors.
 * Only rotors on one plane supported now.
 */
public abstract class AbstractFixedWing extends AbstractVehicle {
    private double dragMove = 0.0;
    private double dragRotate = 0.0;
    protected Rotor[] rotors;
    protected ActuatedWing wing;
    protected Servo2D servo; 
    long t0 = 0;



    public AbstractFixedWing(World world, String modelName, boolean showGui) {
        super(world, modelName, showGui);

        wing = new ActuatedWing();
        servo = new Servo2D();
        rotors = new Rotor[getRotorsNum()];
        for (int i = 0; i < getRotorsNum(); i++) {
            rotors[i] = new Rotor();
        }
    }


    public void report(StringBuilder builder) {
        super.report(builder);
        builder.append("FIXED WING");
        builder.append(newLine);
        builder.append("===========");
        builder.append(newLine);
    }

    private void reportRotor(StringBuilder builder, int rotorIndex) {
    }

    /**
     * Get number of rotors.
     *
     * @return number of rotors
     */
    protected abstract int getRotorsNum();

    /**
     * Get rotor position relative to gravity center of vehicle.
     *
     * @param i rotor number
     * @return rotor radius-vector from GC
     */
    protected abstract Vector3d getRotorPosition(int i);

    public void setDragMove(double dragMove) {
        this.dragMove = dragMove;
    }

    public void setDragRotate(double dragRotate) {
        this.dragRotate = dragRotate;
    }

    @Override
    public void update(long t, boolean paused) {
    
        if (paused) {
            return;
        }

        // Set AirSpeed
        Vector3d airSpeed = new Vector3d(getVelocity());
        Vector3d windSpeed = new Vector3d(0.0,0.0,0.0);


        if (!ignoreWind && armed) {
            windSpeed.add(getWorld().getEnvironment().getCurrentWind(position));
            windSpeed.scale(-1.0);
        }

        airSpeed.add(windSpeed);

        wing.setAirSpeed(airSpeed);
        wing.setArmed(armed);

        // Pass vehcile state to wing
        wing.setVehicleDynamics(
            getPosition(), 
            getVelocity(), 
            getAcceleration(), 
            getRotationRate(), 
            getRotation()
        );

        // Update loop
        wing.update(t, paused);
        servo.update(t, paused);
        for (Rotor rotor : rotors) {
            rotor.update(t, paused);
        }
        super.update(t, paused);

        int nRotors = rotors.length;

        // Rotor throttles
        for (int i = 0; i < nRotors; i++) {
            double c = control.size() > i ? control.get(i) : 0.0;
            rotors[i].setControl(c);
        }

        // Servo commands (last two channels)
        double pitchCmd = control.size() > nRotors     ? control.get(nRotors)     : 0.0;
        double yawCmd   = control.size() > nRotors + 1 ? control.get(nRotors + 1) : 0.0;
        servo.setCommand(pitchCmd, yawCmd);

        wing.setControl(control); // Unused

        // System.out.println(
        //     "pitch =" + servo.getPitch() + ", " +
        //     "yaw=" + servo.getYaw()
        // );

        if (t - t0 >= 1.0) {
            System.out.printf(" pitch=%.2f, yaw=%.2f %n",
                servo.getPitch()*180/3.14,
                servo.getYaw()*180/3.14
            );

            t0 = t;
        }
      

        // System.out.println(
        //     "size: " + control.size() + ", " +
        //     "control[0]=" + (control.size() > 0 ? control.get(0) : 0.0) + ", " +
        //     "control[1]=" + (control.size() > 1 ? control.get(1) : 0.0) + ", " +
        //     "control[2]=" + (control.size() > 2 ? control.get(2) : 0.0) + ", " +
        //     "control[2]=" + (control.size() > 3 ? control.get(3) : 0.0) + ", " +
        //     "control[3]=" + (control.size() > 4 ? control.get(4) : 0.0)
        // );

    }

    @Override
    protected Vector3d getForce() {

        // ---Gimbal Direction ---
        double pitch = servo.getPitch();
        double yaw   = servo.getYaw();
        Vector3d dir = getThrustDirection(pitch, yaw);

        int n = getRotorsNum();
        Vector3d fBody = new Vector3d();

        // --- Aerodynamic body forces (already in body frame) ---
        fBody.set(wing.getThrust());

        for (int i = 0; i < n; i++) {
            double T = rotors[i].getThrust();

            Vector3d fRotor = new Vector3d(dir);
            fRotor.scale(T);

            fBody.add(fRotor);
        }

        // --- Rotate body → NED ---
        rotation.transform(fBody);

        return fBody;
    }

    @Override
    protected Vector3d getTorque() {

        // ---Gimbal Direction ---
        double pitch = servo.getPitch();
        double yaw   = servo.getYaw();
        Vector3d dir = getThrustDirection(pitch, yaw);

        int n = getRotorsNum();
        Vector3d torque = new Vector3d();

        // Aerodynamic torques
        torque.add(wing.getTorque());

        // Add thrust vectored propeller forces

        for (int i = 0; i < n; i++) {

            // Due to arm effects
            Vector3d r = getRotorPosition(i);

            Vector3d F = new Vector3d(dir);
            F.scale(rotors[i].getThrust());

            Vector3d t = new Vector3d();
            t.cross(r, F);
            torque.add(t);

            // due to propeller effects
            Vector3d M = new Vector3d(dir);
            M.scale(rotors[i].getTorque());
            torque.add(M);

        }

        return torque;
    }


    private Vector3d getThrustDirection(double pitch, double yaw) {
        return new Vector3d(
            Math.cos(pitch) * Math.cos(yaw),
            Math.sin(yaw),
            -Math.sin(pitch) * Math.cos(yaw)
        );
    }

    protected Vector3d getAirFlowForce(Vector3d airSpeed) {
        Vector3d f = new Vector3d(airSpeed);
        f.scale(f.length() * dragMove);
        return f;
    }

    protected Vector3d getAirFlowTorque(Vector3d airRotationRate) {
        Vector3d f = new Vector3d(airRotationRate);
        f.scale(f.length() * dragRotate);
        return f;
    }

}
