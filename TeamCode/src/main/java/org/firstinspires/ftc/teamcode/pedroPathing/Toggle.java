package org.firstinspires.ftc.teamcode.pedroPathing;

public class Toggle {

    boolean lastMovement = false;
    boolean toggle = false;

    public boolean update(boolean currentState) {
        if (currentState && !lastMovement) {
            toggle = !toggle;
        }
        lastMovement = currentState;
        return toggle;
    }
}