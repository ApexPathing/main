package paths.builders;

import paths.movements.FollowerMovement;

/**
 * A generic interface for all navigational builders.
 * * @param <T> The specific type of FollowerMovement this builder constructs (e.g., Path, Turn).
 */
public interface MovementBuilder<T extends FollowerMovement> {

    /**
     * Compiles all configuration data and returns the executable movement.
     * @return The constructed FollowerMovement.
     */
    T build();
}