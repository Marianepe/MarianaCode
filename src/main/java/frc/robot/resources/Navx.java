package frc.robot.resources;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Navx {
    private static double angle;
    private double lastAngle;
    private double rate;
    private double pitch;
    private double acceleration;
  
    private double last_world_linear_accel_x;
    private double last_world_linear_accel_y;
    private boolean collisionDetected = false;
    private double curr_world_linear_accel_x;
    private double curr_world_linear_accel_y;
    private double currentJerkX;
    private double currentJerkY;
    private double kCollisionThreshold_DeltaG = 1.5f;


    public Navx() {
        try {
            //ahrs = new AHRS(SPI.Port.kMXP);
            DriverStation.reportWarning("Navx running", true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error stantiating navX-MXP:  " + ex.getMessage(), true);
        }
        last_world_linear_accel_x = 0.0f;
        last_world_linear_accel_y = 0.0f;
    }

 

    public static double getGyro() {
        double difference;
        return angle + 180;
    }

    public double getMagnitud() {
        return rate;
    }

    public double getAcceleration() {
        return acceleration;
    }

   

    public double getPitch() {
        return pitch;
    }

   

   

}

