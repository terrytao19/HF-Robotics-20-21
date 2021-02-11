package org.firstinspires.ftc.teamcode.Misc;

public class ButtonToggle {
    private boolean lastState;
    private boolean toggle;

    //switches between two different states by pressing a button
    public boolean getState(boolean state) {
        if (state && !lastState)
            toggle = !toggle;
        lastState = state;
        return toggle;
    }

    //detects when a button goes from not pressed to pressed
    public boolean buttonPressed(boolean state){
        if(state && !lastState){
            lastState = state;
            return true;
        }
        lastState = state;
        return false;
    }
}
