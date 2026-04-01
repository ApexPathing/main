package controllers;

abstract class Controller {
    private Double goal = 0.0;

    public void  setGoal(double newGoal) {
        this.goal = newGoal;
    }

    abstract double calculate(double currentPosition);
}