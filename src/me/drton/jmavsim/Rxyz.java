package me.drton.jmavsim;

import javax.vecmath.Vector3d;
import javax.vecmath.Matrix3d;

public class Rxyz {

    /**
     * Creates a rotation matrix using roll (X), pitch (Y), and yaw (Z) in radians.
     * Order of rotations: yaw → pitch → roll (Z-Y-X)
     */
    public static Matrix3d createRotationMatrix(double roll, double pitch, double yaw) {
        Matrix3d rollMatrix = new Matrix3d();
        Matrix3d pitchMatrix = new Matrix3d();
        Matrix3d yawMatrix = new Matrix3d();

        // Set individual rotations
        rollMatrix.setIdentity();
        rollMatrix.rotX(roll);

        pitchMatrix.setIdentity();
        pitchMatrix.rotY(pitch);

        yawMatrix.setIdentity();
        yawMatrix.rotZ(yaw);

        // Combine in Z-Y-X order: R = roll * pitch * yaw

        // Matrix3d rotationMatrix = new Matrix3d();
        // rotationMatrix.mul(pitchMatrix, yawMatrix);   // temp = pitch * yaw
        // rotationMatrix.mul(rollMatrix, rotationMatrix); // R = roll * (pitch * yaw)

        Matrix3d rotationMatrix = new Matrix3d();
        rotationMatrix.mul(pitchMatrix, rollMatrix);   // temp = pitch * yaw
        rotationMatrix.mul(yawMatrix, rotationMatrix); // R = roll * (pitch * yaw)


        return rotationMatrix;
    }
} 
