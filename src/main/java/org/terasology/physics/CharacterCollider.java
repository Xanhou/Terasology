package org.terasology.physics;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

/**
 * This class provides an interface for interaction with colliders of the
 * physics engine and is meant for character movement.
 *
 * @author Xanhou
 */
public interface CharacterCollider {
    
    /**
     * Retrieves the location of this body.
     * @param out output parameter to put the results in.
     * @return out, for easy of use.
     */
    public Vector3f getLocation(Vector3f out);
    
    /**
     * Retrieves the orientation of this body.
     * @param out output parameter to put the results in.
     * @return out, for easy of use.
     */
    public Quat4f getOrientation(Quat4f out);
    
    /**
     * Sets the orientation of this body.
     * @param orientation 
     */
    public void setOrientation(Quat4f orientation);
    
    /**
     * Sets the worlds location of this body.
     * @param loc 
     */
    public void setLocation(Vector3f loc);
    
    /**
     * Possibly more efficient method to set both orientation and location at
     * the same time.
     *
     * @param loc
     * @param orientation
     */
    public void setTransform(Vector3f loc, Quat4f orientation);
    
    public SweepCallback sweep(Vector3f startPos, Vector3f endPos, float allowedPenetration, float slopeFactor);
}
