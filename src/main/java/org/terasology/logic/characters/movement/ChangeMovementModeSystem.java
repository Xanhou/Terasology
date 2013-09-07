package org.terasology.logic.characters.movement;

import org.terasology.entitySystem.RegisterMode;
import org.terasology.entitySystem.systems.ComponentSystem;
import org.terasology.entitySystem.systems.RegisterSystem;

/**
 *
 * @author XanHou
 */
@RegisterSystem(RegisterMode.ALWAYS)
public class ChangeMovementModeSystem implements ComponentSystem{

    @Override
    public void initialise() {
    }

    @Override
    public void shutdown() {
    }
    
}
