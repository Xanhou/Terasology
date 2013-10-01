package org.terasology.logic.characters.movement;

import org.terasology.entitySystem.event.Event;
import org.terasology.logic.characters.CharacterStateEvent;

/**
 * This event is used by character movers to request that the System in control
 * of changing the movement mode does what is is supposed to do: change the
 * mode.
 * </p>
 * This event is different from the events in the character.events package in 
 * that it does not indicate a certain action or movement, but is a request to
 * a system to do something.
 * 
 * @author XanHou
 */
public class UpdateMovementModeEvent implements Event{
    private final CharacterStateEvent initial;
    private final CharacterStateEvent result;
    private final boolean firstRun;
    
    public UpdateMovementModeEvent(CharacterStateEvent initialState, CharacterStateEvent resultState, boolean firstRun) {
        initial = initialState;
        result = resultState;
        this.firstRun = firstRun;
    }
    
    /**
     * Returns the state of the character before moving it. Make sure not to
     * alter the fields of this object. It is meant as read-only.
     * @return The initial state of the character before moving it.
     */
    public CharacterStateEvent getInitialState() {
        return initial;
    }
    
    /**
     * Any result after computing the new movement mode can and should be set
     * into this CharacterStateEvent.
     *
     * @return The resulting state of the movement.
     */
    public CharacterStateEvent getResultState() {
        return result;
    }
    
    /**
     * @see CharacterMoveInputEvent.isFirstRun()
     * @return true if this is the first run of ... TODO: document this properly.
     */
    public boolean isFirstRun() {
        return firstRun;
    }
}
