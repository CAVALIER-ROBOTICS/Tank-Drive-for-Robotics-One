// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class DriveTrainState {
    private double leftPercent;
    private double rightPercent;

    public DriveTrainState(double leftPercent, double rightPercent) {
        this.leftPercent = leftPercent;
        this.rightPercent = rightPercent;
    }

    public double getLeftPercent() {
        return leftPercent;
    }

    public double getRightPercent() {
        return rightPercent;
    }

    public static DriveTrainState zeroes() {
        return new DriveTrainState(0, 0);
    }
}
