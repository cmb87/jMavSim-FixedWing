package me.drton.jmavsim.vehicle;

import me.drton.jmavsim.World;
import me.drton.jmavsim.Rotor;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Generic quadcopter model.
 */
public class CoaxialTailsitter extends AbstractFixedWing {

    private static final int rotorsNum = 2;
    private Vector3d[] rotorPositions = new Vector3d[rotorsNum];
    private int[] rotorRotations = new int[rotorsNum];

    /**
     * Generic quadcopter constructor.
     *
     * @param world          world where to place the vehicle
     * @param modelName      filename of model to load, in .obj format
     * @param orientation    "x" or "+"
     * @param style          rotor position layout style. "default"/"px4" for px4, or "cw_fr" CW sequential layout starting at front motor
     * @param armLength      length of arm from center [m]
     * @param rotorThrust    full thrust of one rotor [N]
     * @param rotorTorque    torque at full thrust of one rotor in [Nm]
     * @param rotorTimeConst spin-up time of rotor [s]
     * @param rotorsOffset   rotors positions offset from gravity center
     * @param showGui        false if the GUI has been disabled
     */
    public CoaxialTailsitter(World world, String modelName, String orientation, String style,
                      double armLength, double rotorThrust, double rotorTorque,
                      double rotorTimeConst, Vector3d rotorsOffset, boolean showGui) {
        super(world, modelName, showGui);


        //rotorPositions[0] = new Vector3d(-armLength, 0.0, 0.0);
       // rotorPositions[1] = new Vector3d(-armLength, 0.0, 0.0);

        // Define rotor positions FIRST
        rotorPositions[0] = new Vector3d(-armLength, 0.0, 0.0);
        rotorPositions[1] = new Vector3d(-armLength, 0.0, 0.0);

        // Rotor spin directions
        rotorRotations[0] = 1;
        rotorRotations[1] = -1;

        for (int i = 0; i < rotorsNum; i++) {

            // Apply offset safely
            rotorPositions[i].add(rotorsOffset);

            Rotor rotor = rotors[i];
            rotor.setFullThrust(rotorThrust);
            rotor.setFullTorque(rotorTorque * rotorRotations[i]);
            rotor.setTimeConstant(rotorTimeConst);
        }

    }
    @Override
    protected int getRotorsNum() {
        return rotorsNum;
    }

    @Override
    protected Vector3d getRotorPosition(int i) {
        return rotorPositions[i];
    }
}
