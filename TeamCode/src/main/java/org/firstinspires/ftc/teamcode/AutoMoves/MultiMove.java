package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

public class MultiMove extends AutoModeMovements{

    AutoModeMovements[] moves;

    public MultiMove(AutoModeMovements[] moves){
        this.moves = moves;
    }


    @Override
    public boolean isDone() {
        for(AutoModeMovements m: moves){
            if(!m.isDone())
                return false;
        }
        return true;
    }

    @Override
    public void doMovement() {
        for(AutoModeMovements m: moves){
            if(!m.isDone()){
                m.doMovement();
            }
        }
    }

    @Override
    public void onStart() {
        for(AutoModeMovements m: moves){
            if(!m.isDone()){
                m.onStart();
            }
        }
    }

    @Override
    public void onEnd() {
        for(AutoModeMovements m: moves){
            m.onEnd();
        }
    }

    @Override
    public Pair<String, String> getStatus() {
        String returnString = "";
        for(AutoModeMovements m: moves){
            Pair<String, String> mStatus = m.getStatus();
            returnString = returnString + "\n * " + mStatus.first + " | " + mStatus.second;
        }

        return new Pair<>("MultiAction", returnString);
    }
}