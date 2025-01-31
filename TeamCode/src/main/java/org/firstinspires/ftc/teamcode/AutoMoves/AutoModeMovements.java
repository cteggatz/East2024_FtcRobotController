package org.firstinspires.ftc.teamcode.AutoMoves;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MotorData;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Queue;

/**
 * An interface to describe a movement the bot can do
 */
public abstract class AutoModeMovements {
    protected boolean isWorking = false;

    public abstract boolean isDone();
    public abstract void doMovement();
    public abstract void onStart();
    public abstract void onEnd();
    public abstract Pair<String, String> getStatus();


    protected void setWorking(boolean val){
        isWorking = val;
    }
    public boolean isWorking(){
        return this.isWorking;
    }
    /**
     * If you are using a linear op mode, this will just do the entire movement from start to finish with one function call
     */
    public void doLinearMove(){
        onStart();
        while(!isDone()){
            doMovement();
        }
        onEnd();
    }
}








