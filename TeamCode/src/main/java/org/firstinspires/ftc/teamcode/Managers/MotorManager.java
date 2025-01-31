package org.firstinspires.ftc.teamcode.Managers;

import com.qualcomm.robotcore.util.Range;

public class MotorManager {
    private double rotation = 0;
    private double gearRatio = 1;
    private double mult = 1;
    private int counts;
    private boolean override;

    private double targetPower = 0;
    private double targetRotation = 0;
    private double targetCutoff = 0;
    private boolean hasMaintain = false;
    private boolean hasSetTarget = false;

    private double min = 0;
    private double minCutoff;
    private boolean hasMin = false;
    private double max = 0;
    private double maxCutoff;
    private boolean hasMax = false;
    public MotorManager(int counts) {
        this.counts = counts;
    }

    public MotorManager UsingGearIncrease(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public MotorManager UsingGearReduction(double gearRatio) {
        this.gearRatio = 1/gearRatio;
        return this;
    }

    public MotorManager UsingRawRevolutions() {
        this.mult = 1.0;
        return this;
    }

    public MotorManager UsingCounts() {
        this.mult = 1.0/counts;
        return this;
    }

    public MotorManager UsingRevolutions() {
        this.mult = gearRatio;
        return this;
    }

    public MotorManager UsingDegrees() {
        this.mult = gearRatio/360;
        return this;
    }

    public MotorManager UsingRadians() {
        this.mult = gearRatio/(2*Math.PI);
        return this;
    }

    public MotorManager Min(double min) {
        this.min = min * mult;
        this.hasMin = true;
        return this;
    }

    public MotorManager Min(double min, double cutoff) {
        this.minCutoff = cutoff * mult;
        return Min(min);
    }

    public MotorManager Max(double max) {
        this.max = max * mult;
        this.hasMax = true;
        return this;
    }

    public MotorManager Max(double max, double cutoff) {
        this.maxCutoff = cutoff * mult;
        return Max(max);
    }

    public MotorManager AutoError(double cutoff) {
        this.targetCutoff = cutoff * mult;
        return this;
    }

    public MotorManager Maintain() {
        this.hasMaintain = true;
        return this;
    }

    public MotorManager Maintain(double cutoff) {
        return this
                .Maintain()
                .AutoError(cutoff);
    }

    public MotorManager DisableMin() {
        this.hasMin = false;
        return this;
    }

    public MotorManager DisableMax() {
        this.hasMax = false;
        return this;
    }

    public MotorManager DisableMaintain() {
        this.hasMaintain = false;
        return this;
    }

    public void EnableOverride() {
        this.override = true;
        ResetTargetRotation();
    }

    public void DisableOverride() {
        this.override = false;
    }

    public void UpdateRotation(double rotation) {
        this.rotation = rotation * mult;
    }

    public void SetTargetPower(double power) {
        this.targetPower = power;
        if (hasMaintain && power != 0)
            SetTargetRotation(GetRotation());

    }

    public void SetTargetRotation(double target) {
        targetRotation = target * mult;
        hasSetTarget = true;
    }

    public void ResetTargetRotation() {
        targetRotation = rotation;
        hasSetTarget = false;
    }

    public boolean IsNearTargetRotation() {
        return hasSetTarget && Math.abs(targetRotation-rotation) < targetCutoff;
    }

    public double GetFinalPower() {
        double power = targetPower;

        if (power == 0 && hasSetTarget)
            power = soften(targetRotation-rotation, targetCutoff);

        if (power > 0 && hasMax && !override)
            power *= soften(max-rotation,maxCutoff);

        if (power < 0 && hasMin && !override)
            power *= soften(rotation-min,minCutoff);

        return Range.clip(power, -1, 1);
    }

    public double GetRotation() {
        return rotation / mult;
    }

    public double GetTargetRotation() {
        return targetRotation / mult;
    }

    public double FromProportionalRotation(double proportion) {
        if (!hasMin || !hasMax) throw new IllegalStateException("Minimum and Maximum must be defined.");
        return (min + (max-min) * proportion) / mult;
    }

    public boolean HasTargetRotation() {
        return hasSetTarget;
    }

    public boolean IsOverrideEnabled() {
        return override;
    }

    /**
     * @return the min count of the manager
     * @author Christopher Teggatz
     */
    public double GetMin(){
        return this.min / mult;
    }

    /**
     * @return the max count of the manager
     * @author Christopher Teggatz
     */
    public double GetMax(){
        return this.max / mult;
    }

    private double soften(double difference, double cutoff) {
        if (Math.abs(difference) >= cutoff) return Math.signum(difference);
        return difference / cutoff;
    }
}
