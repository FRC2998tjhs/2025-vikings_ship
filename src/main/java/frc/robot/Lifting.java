package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Lifting {
    public final Solenoid left;
    public final Solenoid right;
    public final Solenoid rear;

    Lifting(Solenoid left, Solenoid right, Solenoid rear) {
        this.left = left;
        this.right = right;
        this.rear = rear;
    }

    public void onStart() {
        set(left, Status.NORMAL);
        set(right, Status.NORMAL);
    }

    private void set(Solenoid solenoid, Status status) {
        solenoid.set(status == Status.NORMAL);
    }

    public void releaseArms() {
        set(left, Status.INVERTED);
        set(right, Status.INVERTED);
    }

    public void liftRobot() {
        set(left, Status.NORMAL);
        set(right, Status.NORMAL);
    }

    enum Status {
        NORMAL, INVERTED
    }
}
