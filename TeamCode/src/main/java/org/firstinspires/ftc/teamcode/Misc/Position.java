package org.firstinspires.ftc.teamcode.Misc;

import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Angle;
import org.firstinspires.ftc.teamcode.RobotStuff.diffyswerve.Vector2d;

public class Position {
    //absolute position on the field
    double x;
    double y;
    Angle heading;

    public Position (double x, double y, Angle heading){
        this.x = x;
        this.y = y;
        this.heading = heading;

    }
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public void increment (double deltaX, double deltaY, double deltaHeading) {
        this.x += deltaX;
        this.y += deltaY;
        this.heading = new Angle(heading.getAngle() = deltaHeading, heading.getType());
    }
    public void incrementHeading (double deltaHeading){
        heading = heading.rotateBy(deltaHeading, Angle.Direction.CLOCKWISE);

    }
    public boolean Range (Position otherPosition, double MaxErrorx, double MaxErrory, double headingMaxError){
        double xError = getAbsXDifference(otherPosition);
        double yError = getAbsYDifference(otherPosition);
        double headingError = getAbsHeadingDifference(otherPosition);
        return xError < MaxErrorx && yError < MaxErrory && headingError < headingMaxError;

    }
    public double getAbsXDifference (Position otherPosition) {
        return Math.abs(this.x - otherPosition.x);

    }
    public double getAbsYDifference(Position otherPosition){
        return Math.abs(this.y - otherPosition.y);

    }
    public double getAbsHeadingDifference(Position otherPosition) {
        return this.heading.getDifference(otherPosition.heading);
    }
    public double getSignedHeadingDifference(Position otherPosition) {
        double difference = this.heading.getDifference(otherPosition.heading);
        if (this.heading.directionTo(otherPosition.heading) == Angle.Direction.COUNTER_CLOCKWISE) {
            return difference * -1; //todo: check sign
        }
        return difference;
    }
    public Vector2d getDirectionTo (Position targetPosition) {
        return getVectorTo(targetPosition).getUnitVector();
    }
    public Angle.Direction getRotationDirectionTo (Position targetPosition) {
        return this.heading.directionTo(targetPosition.heading);
    }

}
