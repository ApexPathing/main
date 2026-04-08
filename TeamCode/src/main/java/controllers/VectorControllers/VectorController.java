package controllers.VectorControllers;

import util.Vector;

/**
 * Vector controllers allow more accuracy for long trajectories. Since some feedback controllers
 * (i.e. SquID) have non-linear behavior, this can result in undesirable behavior in the form of
 * curved trajectories. This fixes that issue by pointing all the feedback power straight in the
 * direction of the error.
 * <p>
 * Author: DrPixelCat24 (7842 alum)
 */
public abstract class VectorController {
    protected volatile Vector goal = new Vector();

    public void setGoal(Vector newGoal) {
        this.goal = newGoal;
    }

    abstract Vector calculate(Vector currentPosition);
}