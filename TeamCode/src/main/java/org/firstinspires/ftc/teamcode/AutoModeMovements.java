package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

/**
 * An interface to describe a movement the bot can do
 */
public abstract class AutoModeMovements {
    protected boolean isWorking = false;

    public abstract boolean isDone();
    public abstract void doMovement();
    public abstract Pair<String, String> getStatus();


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

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("","");
    }
}

class PivotArm extends AutoModeMovements{
    private DcMotor pivotMotor;
    int targetPosition;
    float direction;


    public PivotArm(DcMotor pivotMotor, int targetPosition){
        this.pivotMotor = pivotMotor;
        this.targetPosition = targetPosition;

        //direction = ((pivotMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);
    }



    @Override
    public boolean isDone() {
        //telemetry.addData("isDone", "" + Math.abs(targetPosition - pivotMotor.getCurrentPosition()));
        return Math.abs(targetPosition - pivotMotor.getCurrentPosition()) <= MotorData.PIVOT_ERROR_COUNT;
    }

    @Override
    public void doMovement() {
        direction = ((pivotMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);
        int pivotPosition = pivotMotor.getCurrentPosition();
        double pivotPower = 0;

        pivotPower += direction * MotorData.PIVOT_SPEED_MULT;

        if (MotorData.MAX_COUNT-pivotPosition < MotorData.CUTOFF_COUNT && pivotPower > 0 && MotorData.CUTOFF_PROPORTION > 0) {
            pivotPower *= (MotorData.MAX_COUNT-pivotPosition)/((double)MotorData.CUTOFF_COUNT);
        }

        if (pivotPosition-MotorData.MIN_COUNT < MotorData.CUTOFF_COUNT && pivotPower < 0 && MotorData.CUTOFF_PROPORTION > 0) {
            pivotPower *= (pivotPosition-MotorData.MIN_COUNT)/((double)MotorData.CUTOFF_COUNT);
        }

        pivotMotor.setPower(Range.clip(pivotPower,-1,1) * .5);
       // telemetry.addData("AutoMove", "Moving Pivot");
    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("Pivot Movement", "Current Pos: " +  pivotMotor.getCurrentPosition() + "\nTarget Pos: " + this.targetPosition);
    }

    private int getTargetPosition(){
        return this.targetPosition;
    }
}

class Pause extends AutoModeMovements{
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
    public Pair<String, String> getStatus() {
        return new Pair<>("Timer Movement", "Target Time: " + endTime + "\nCurrent Time: " + timer.milliseconds());
    }
}

class ExtendLift extends  AutoModeMovements{
    DcMotor liftMotor;
    int targetPosition;
    float direction;

    public ExtendLift(DcMotor liftMotor, int targetPosition){
        this.liftMotor = liftMotor;
        this.targetPosition = targetPosition;
    }


    @Override
    public boolean isDone() {
        return Math.abs(targetPosition - liftMotor.getCurrentPosition()) <= MotorData.LIFT_ERROR;
    }

    @Override
    public void doMovement() {
        direction = ((liftMotor.getCurrentPosition() >= targetPosition) ? -1 : 1);

        float currentPosition = liftMotor.getCurrentPosition();
        if(currentPosition <= MotorData.LIFT_MAX_ROTATION && currentPosition >= MotorData.LIFT_MIN_ROTATION){
            liftMotor.setPower(direction);
        }

    }

    @Override
    public Pair<String, String> getStatus() {
        return new Pair<>("Extend Lift Movement", "Current Pos: " + liftMotor.getCurrentPosition() + "\nTargetPos: " + this.targetPosition);
    }
}
