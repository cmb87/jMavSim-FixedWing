package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.ReportUtil;
import me.drton.jmavsim.Rotor;
import me.drton.jmavsim.ActuatedWing;
import me.drton.jmavsim.World;

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

    public AbstractFixedWing(World world, String modelName, boolean showGui) {
        super(world, modelName, showGui);

        wing = new ActuatedWing();
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

        // System.out.println(
        //     "size: " + control.size() + ", " +
        //     "control[0]=" + (control.size() > 0 ? control.get(0) : 0.0) + ", " +
        //     "control[1]=" + (control.size() > 1 ? control.get(1) : 0.0) + ", " +
        //     "control[2]=" + (control.size() > 2 ? control.get(2) : 0.0) + ", " +
        //     "control[2]=" + (control.size() > 3 ? control.get(3) : 0.0) + ", " +
        //     "control[3]=" + (control.size() > 4 ? control.get(4) : 0.0)
        // );



        wing.update(t, paused);
        super.update(t, paused);
        wing.setControl(control);

    }

    @Override
    protected Vector3d getForce() {
   
        Vector3d f = new Vector3d();
        f.set(wing.getThrust());
        rotation.transform(f); //Rotate2NED?

        // System.out.println("fx=" + f.x +
        //                 ", fy=" + f.y +
        //                 ", fz=" + f.z);



        return f;
    }
    

    @Override
    protected Vector3d getTorque() {

        return wing.getTorque();
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
