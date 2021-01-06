package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.util.Range;
/*
Framework for all PID loops.
Instead of writing a new method, make a new instance of the PID class.
*/
public class PID
{
    private double target, current, Kp, Ki, Kd, totalError, KiMax, currentError, lastError, outputValue, previousTime, currentTime;

    //Constructor
    public PID(double Kp, double Ki, double Kd, double KiMax)
    {

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.KiMax = KiMax;

        current = 0;
        lastError = 0;
        totalError = 0;
        previousTime = 0;
    }
    //Constructor without total error constraints
    public PID(double Kp, double Ki, double Kd)
    {

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        current = 0;
        lastError = 0;
        totalError = 0;
        previousTime = 0;
    }
    //Takes in the current value and uses methods P, I, and D to get an output value.
    public void updatePID(double currentValue)
    {
        //current time in seconds
        currentTime = System.currentTimeMillis() / 1000.0;

        current = currentValue;
        outputValue = P() + I() + D();

        lastError = currentError();
        totalError += currentError();

        previousTime = currentTime;
    }

    //Sets the target value
    public void setTarget(double targetValue)
    {
        target = targetValue;
    }

    //Gets the difference between the target value and the current value
    public double currentError(){ return target - current; }

    //Gets the total error for the entire action
    public double totalError()
    {
        return totalError;
    }

    //Gets the difference between the error from the last loop and the error from this loop
    public double errorDifference() {

        return (currentError() - lastError);
    }

    //Update the P, I, and D values
    public double P()
    {
        return Kp * (target - current);
    }
    public double I()
    {
        return Range.clip(Ki * totalError(), -KiMax, KiMax);
    }
    public double D()
    {
        double dt = currentTime - previousTime;
        return Kd * (errorDifference() / dt);
    }

    //Accessor methods for variables
    public double getCurrent()
    {
        return current;
    }
    public double getTarget()
    {
        return target;
    }
    public double getOutput()
    {
        return outputValue;
    }

    public void reset(){
        lastError = 0;
        totalError = 0;
    }

}
