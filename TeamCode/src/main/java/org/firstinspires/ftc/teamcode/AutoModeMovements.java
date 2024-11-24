package org.firstinspires.ftc.teamcode;

/**
 * An interface to describe a movement the bot can do
 */
public abstract class AutoModeMovements {
    protected boolean isWorking = false;

    public abstract boolean isDone();
    public abstract void doMovement();

    protected void setWorking(boolean val){
        isWorking = val;
    }
    public boolean isWorking(){
        return this.isWorking;
    }
}

/**
 * A class that describes physically moving the robot to a position
 */
class MoveBot extends AutoModeMovements{

    float targetPosition;

    public MoveBot(float targetPosition){

    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void doMovement() {

    }
}
