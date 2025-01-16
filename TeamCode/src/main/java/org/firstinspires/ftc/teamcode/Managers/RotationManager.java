package org.firstinspires.ftc.teamcode.Managers;

import com.qualcomm.robotcore.util.Range;

public class RotationManager {
    private double rotation = 0;
    private double gearRatio = 1;
    private double mult = 1;

    private double targetPower = 0;
    private double targetRotation = 0;
    private double autoCutoff = 0;
    private boolean hasAuto = false;
    private boolean hasSetPower = false;

    private double min = 0;
    private double minCutoff;
    private boolean hasMin = false;
    private double max = 0;
    private double maxCutoff;
    private boolean hasMax = false;

    public RotationManager() {

    }

    public RotationManager UsingRevolutions() {
        this.mult = 1.0;
        return this;
    }

    public RotationManager UsingCounts(int counts) {
        this.mult = 1.0/counts;
        return this;
    }

    public RotationManager UsingDegrees() {
        this.mult = 1.0/360;
        return this;
    }

    public RotationManager UsingRadians() {
        this.mult = 1.0/(2*Math.PI);
        return this;
    }

    public RotationManager UsingGearIncrease(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public RotationManager UsingGearReduction(double gearRatio) {
        this.gearRatio = 1/gearRatio;
        return this;
    }

    public RotationManager Min(double min, double cutoff) {
        this.min = min * mult;
        this.minCutoff = cutoff * mult;
        this.hasMin = true;
        return this;
    }

    public RotationManager Max(double max,double cutoff) {
        this.max = max * mult;
        this.maxCutoff = cutoff * mult;
        this.hasMax = true;
        return this;
    }

    public RotationManager Auto(double cutoff) {
        this.autoCutoff = cutoff;
        this.hasAuto = true;
        return this;
    }

    public void UpdateRotation(double rotation) {
        this.rotation = rotation * gearRatio * mult;
    }

    public void SetTargetPower(double power) {
        this.targetPower = power;
        this.targetRotation = rotation;
        this.hasSetPower = true;
    }

    public double GetFinalPower(boolean override) {
        double power = targetPower;

        if (power > 0 && hasMax && !override) {
            power *= lerp(max-rotation,maxCutoff);
        }

        if (power < 0 && hasMin && !override) {
            power *= lerp(rotation-min,minCutoff);
        }

        if (power == 0 && hasAuto && hasSetPower) {
            power = lerp(targetRotation-rotation,autoCutoff);
        }

        return Range.clip(power, -1, 1);

    }

    public double GetRotation() {
        return rotation / mult;
    }

    private double lerp(double difference, double cutoff) {
        if (Math.abs(difference) >= cutoff) return 1;
        return difference / cutoff;
    }
}
