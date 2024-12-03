package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

/**
 * A class that describes physically moving the robot to a position
 */
public class MoveBot extends AutoModeMovements{

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

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {

    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("","");
    }
}
