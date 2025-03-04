package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lifting {
    public final Solenoid left;
    public final Solenoid right;
    public final DoubleSolenoid front;

    Lifting(Solenoid left, Solenoid right, DoubleSolenoid front) {
        this.left = left;
        this.right = right;
        this.front = front;
    }

    public void onStart() {
        set(left, Status.NORMAL);
        set(right, Status.NORMAL);
        front.set(Value.kForward);
    }

    private void set(Solenoid solenoid, Status status) {
        if (solenoid == null) return;
        solenoid.set(status == Status.INVERTED);
    }

    public void drop() {
        set(left, Status.INVERTED);
        set(right, Status.NORMAL);
        front.set(Value.kReverse);
    }

    public void liftRobot() {
        set(left, Status.NORMAL);
        set(right, Status.NORMAL);
    }

    public void coralGrab() {
        set(left, Status.INVERTED);
        set(right, Status.INVERTED);
    }

    public void prepForReset() {
        set(left, Status.NORMAL);
        set(right, Status.INVERTED);
        front.set(Value.kForward);
    }

    enum Status {
        NORMAL, INVERTED
    }
}
