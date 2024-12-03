package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Pause extends AutoModeMovements{
    private ElapsedTime timer;
    private float endTime;

    public Pause(float endTime){
        //timer = new ElapsedTime();
        this.endTime = endTime;
    }


    @Override
    public boolean isDone() {
        return timer != null && timer.milliseconds() >= endTime;
    }

    @Override
    public void doMovement() {
        if(timer == null){
            timer = new ElapsedTime();
            timer.reset();
        }
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onEnd() {

    }

    @Override
    public Pair<String, String> getStatus() {
        if(timer != null){
            return new Pair<>("Timer Movement", "Target Time: " + endTime + "\nCurrent Time: " + timer.milliseconds());
        } else {
            return new Pair<>("Timer Movement", "Target Time: " + endTime + "\nCurrent Time: NOT STARTED");
        }
    }
}