package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Angle;

public class Acquiringposition {
    //absolute position on the field
    private double x;
    private double y;
    Angle heading;

    public void Acquiringposition (double x, double y, Angle heading){
        this.x = x;
        this.y = y;
        this.heading = heading;

    }
    public void increment (double deltaX, double deltaY, double deltaHeading) {
        this.x += deltaX;
        this.y += deltaY;
        this.heading = new Angle(heading.getAngle() = deltaHeading, heading.getType());
    }
    public void incrementHeading (double deltaHeading){
        heading = heading.rotateBy(deltaHeading, Angle.Direction.CLOCKWISE);

    }
    public boolean Range (Acquiringposition otherPosition, double MaxErrorx, double MaxErrory, double headingMaxError){
        double xError = getAbsXDifference(otherPosition);
        double yError = getAbsYDifference(otherPosition);
        double headingError = getAbsHEadingDifference(otherPosition);
        return xError < MaxErrorx && yError < MaxErrory && headingError < headingMaxError;

    }
    public double getAbsXDifference (Acquiringposition otherPosition) {
        return math.abs(this.x - otherPosition.x);

    }
    public double getAbsYDifference(Acquiringposition otherPosition){
        return math.abs(this.y - otherPosition.y);

    }
    public double getx(){
        return x;
    }
}
