package org.terasology.logic.characters.movement.modes;

import javax.vecmath.Vector3f;

/**
 * Holds the result of some movement.
 * 
 * @author Immortius
 */
public class MoveResult {

    private Vector3f finalPosition;
    private boolean horizontalHit;
    private boolean bottomHit;
    private boolean topHit;

    public MoveResult(Vector3f finalPosition, boolean hitHorizontal, boolean hitBottom, boolean hitTop) {
        this.finalPosition = finalPosition;
        this.horizontalHit = hitHorizontal;
        this.bottomHit = hitBottom;
        this.topHit = hitTop;
    }

    public Vector3f getFinalPosition() {
        return finalPosition;
    }

    public boolean isHorizontalHit() {
        return horizontalHit;
    }

    public boolean isBottomHit() {
        return bottomHit;
    }

    public boolean isTopHit() {
        return topHit;
    }
}