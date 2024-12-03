package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Queue;

public class QueueMove extends AutoModeMovements{

    Queue<AutoModeMovements> moves;
    AutoModeMovements[] cleanUp;
    AutoModeMovements current;

    public QueueMove(AutoModeMovements[] moves){
        this.moves = new ArrayDeque<>();
        this.moves.addAll(Arrays.asList(moves));
        current = this.moves.poll();
        cleanUp = moves;
    }

    @Override
    public boolean isDone() {
        return moves.isEmpty() && current.isDone();
    }

    @Override
    public void doMovement() {
        if(current.isDone() && !moves.isEmpty()){
            current.onEnd();
            current = moves.poll();
        } else {
            current.doMovement();
        }
    }

    @Override
    public void onStart() {}

    @Override
    public void onEnd() {
        for(AutoModeMovements move: cleanUp){
            move.onEnd();
        }
    }

    @Override
    public Pair<String, String> getStatus() {
        return current.getStatus();
    }
}