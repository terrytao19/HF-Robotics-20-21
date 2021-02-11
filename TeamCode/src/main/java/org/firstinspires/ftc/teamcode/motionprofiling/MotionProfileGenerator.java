package org.firstinspires.ftc.teamcode.motionprofiling;

import Dashboard.RobotConstants;

public class MotionProfileGenerator {
    //constants based on robot
    public static double MAX_VELOCITY; //inches per second
    public static double MAX_ACCELERATION; //inches per second squared

    private double totalDistance;

    public MotionProfileGenerator(double distance){
        totalDistance = distance;
        MAX_ACCELERATION = RobotConstants.MAX_ACCELERATION;
        MAX_VELOCITY = RobotConstants.MAX_VELOCITY;
    }

    public double[] generateMotionProfile(){
        double[] velocities;

        double maxAccelDistance = MAX_VELOCITY * MAX_VELOCITY / (MAX_ACCELERATION);
        //triangular profile
        if(maxAccelDistance >= totalDistance){
            double adjustedVelocity = Math.sqrt(totalDistance * MAX_ACCELERATION);

            double totalTimeSeconds = 2.0 * (adjustedVelocity / MAX_ACCELERATION);
            velocities = new double[(int)(totalTimeSeconds * 1000)];
            for(int i = 0; i < velocities.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //accelerating
                if(i < velocities.length / 2.0){
                    velocities[i] = MAX_ACCELERATION * currentTimeSeconds;
                }

                //decelerating
                else{
                    velocities[i] = adjustedVelocity - MAX_ACCELERATION * (currentTimeSeconds - (totalTimeSeconds / 2.0));
                }

            }
        }
        //trapezoidal profile
        else{
            double totalTimeSeconds = 2 * MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            double timeTwoThirds = MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            velocities = new double[(int)(totalTimeSeconds * 1000)];

            for(int i = 0; i < velocities.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //accelerating
                if(currentTimeSeconds < MAX_VELOCITY / MAX_ACCELERATION)
                    velocities[i] = MAX_ACCELERATION * currentTimeSeconds;
                    //cruising
                else if(currentTimeSeconds < MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY)
                    velocities[i] = MAX_VELOCITY;
                    //decelerating
                else
                    velocities[i] = MAX_VELOCITY - MAX_ACCELERATION * (currentTimeSeconds - timeTwoThirds);
            }
        }
        return velocities;
    }

    public double[] generatePositionProfile(){
        double[] positions;
        double maxAccelDistance = MAX_VELOCITY * MAX_VELOCITY / (MAX_ACCELERATION);
        //triangular profile
        if(maxAccelDistance >= totalDistance){
            int first = 0;
            int second = 0;
            double adjustedVelocity = Math.sqrt(totalDistance * MAX_ACCELERATION);
            //the length of the graph is t, the height of the graph is vmax, the area of the graph is amax
            double totalTimeSeconds = 2 * Math.sqrt(totalDistance / MAX_ACCELERATION);
            positions = new double[(int)(totalTimeSeconds * 1000)];
            for(int i = 0; i < positions.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //accelerating
                if(currentTimeSeconds < totalTimeSeconds / 2){
                    positions[i] = 0.5 * MAX_ACCELERATION * currentTimeSeconds * currentTimeSeconds;
                    first++;
                }
                //decelerating
                else {
                    double dt = currentTimeSeconds - totalTimeSeconds / 2;
                    positions[i] = totalDistance / 2 + adjustedVelocity * dt + 0.5 * -MAX_ACCELERATION * dt * dt;
                    second++;
                }
            }
            System.out.println(first + " " + second);
        }
        //trapezoidal profile
        else{
            double totalTimeSeconds = 2 * (MAX_VELOCITY / MAX_ACCELERATION) + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            double timeTwoThirds = MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            positions = new double[(int)(totalTimeSeconds * 1000)];

            System.out.println("Acceleration distance: " + maxAccelDistance / 2);
            int first = 0, second = 0, third = 0;
            for(int i = 0; i < positions.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //System.out.println(" i: " + i + " ");
                //accelerating
                if(currentTimeSeconds < MAX_VELOCITY / MAX_ACCELERATION){
                    positions[i] = 0.5 * MAX_ACCELERATION * (currentTimeSeconds * currentTimeSeconds);
                }

                //cruising
                else if(currentTimeSeconds < timeTwoThirds){
                    double dt = currentTimeSeconds - MAX_VELOCITY / MAX_ACCELERATION;
                    positions[i] = 0.5 * MAX_VELOCITY * (MAX_VELOCITY / MAX_ACCELERATION) + MAX_VELOCITY * dt;
                }

                //decelerating
                else{
                    double dt = currentTimeSeconds - timeTwoThirds;
                    positions[i] = (totalDistance - maxAccelDistance/2) + MAX_VELOCITY * dt + 0.5 * -MAX_ACCELERATION * dt * dt;
                }
                System.out.println(positions[i]);
            }
        }
        return positions;
    }
}
