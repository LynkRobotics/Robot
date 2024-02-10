package frc.lib.util;

public class ShooterConfiguration {

    private double distance;
    private double topMotorSpeed;
    private double bottomMotorSpeed;

    public ShooterConfiguration(double distance, double topMotorSpeed, double bottomMotorSpeed) {
        this.distance = distance;
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed; 
    }
 
    public double getDistance() {
        return distance;
    }

    public double getTopMotorSpeed() {
        return topMotorSpeed;
    }

    public double getBottomMotorSpeed() {
        return bottomMotorSpeed;
    }

    
}
