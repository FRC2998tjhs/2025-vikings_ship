package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lifting {
    private Solenoid frontHolder;
    private Solenoid middlePush;
    private DoubleSolenoid frontLift;
    private Solenoid rearLiftPush;
    private Solenoid rearLiftPull;

    Lifting(Solenoid frontHolder, Solenoid middlePush, DoubleSolenoid frontLift, Solenoid rearLiftPush,
            Solenoid rearLiftPull) {
        this.frontHolder = frontHolder;
        this.middlePush = middlePush;
        this.frontLift = frontLift;
        this.rearLiftPush = rearLiftPush;
        this.rearLiftPull = rearLiftPull;
    }

    public void start() {
        frontHeld();
        frontNotLifting();
        middleNeutral();
        rearPush();
    }

    public void drop() {
        frontRelease();
        frontNotLifting();
        middlePush();
        rearNeutral();
    }

    public void liftRobot() {
        frontRelease();
        frontLifting();
        middleNeutral();
        rearPush();
    }

    public void coralGrab() {
        rearPull();
        middleNeutral();
    }

    public void reset() {
        frontRelease();
        frontNotLifting();
        middleNeutral();
        rearNeutral();
    }

    private void frontHeld() {
        set(frontHolder, Status.NORMAL);
    }

    private void frontRelease() {
        set(frontHolder, Status.INVERTED);
    }

    private void rearPush() {
        set(rearLiftPush, Status.NORMAL);
        set(rearLiftPull, Status.NORMAL);
    }

    private void rearNeutral() {
        set(rearLiftPush, Status.INVERTED);
        set(rearLiftPull, Status.NORMAL);
    }

    private void rearPull() {
        set(rearLiftPush, Status.INVERTED);
        set(rearLiftPull, Status.INVERTED);
    }

    private void middleNeutral() {
        set(middlePush, Status.NORMAL);
    }

    private void middlePush() {
        set(middlePush, Status.INVERTED);
    }

    private void frontNotLifting() {
        frontLift.set(Value.kReverse);
    }

    private void frontLifting() {
        frontLift.set(Value.kForward);
    }

    private void set(Solenoid solenoid, Status status) {
        if (solenoid == null)
            return;
        solenoid.set(status == Status.INVERTED);
    }

    enum Status {
        NORMAL, INVERTED
    }
}
