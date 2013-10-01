package org.terasology.logic.characters.movement.modes;

import org.terasology.entitySystem.EntityRef;
import org.terasology.logic.characters.CharacterMoveInputEvent;
import org.terasology.logic.characters.CharacterStateEvent;

/**
 *
 * @author Rednax
 */
public interface MovementMode {
    /**
     * Returns the unique name or key for this movement mode. When two movement
     * modes have the same key strange stuff starts to happen, so the implementor
     * of this method is advised to use the module name as a prefix, similar to
     * block names.
     * @return A non null String object that will serve as key to define this
     * movement mode.
     */
    String getKey();
    
    /**
     * Determines the new state for 'character' and writes the result into
     * 'result'. It is also responsible for sending any events that result from
     * this movement, such a collision events or footstep events.
     * </p>
     * It should always hold that result.getMode().equals(this.getKey())
     *
     * @param character The entity to update the state of.
     * @param result Set to the initial state before before movement, except for
     * the sequence number which is incremented before calling this method. This
     * method is expected to change the position, rotation, velocity, grounded
     * and footstepDelta field of the state. The mode should be set by a System
     * listening to the UpdateMovementModeEvent event and the remaining fields
     * are not dependant on the type of movement and are set by the
     * CharacterMover.
     * @param input The user or AI input to base the movement on.
     */
    void updateState(EntityRef character, CharacterStateEvent result,
            CharacterMoveInputEvent input);
}
