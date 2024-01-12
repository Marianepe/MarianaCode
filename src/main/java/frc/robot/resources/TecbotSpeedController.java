/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.*;

/**
 * Creates new TecbotSpeedController and allows a generalized
 * use of speed controllers. To see the currently supported
 * motor controllers you can check {@link TypeOfMotor}
 */
public class TecbotSpeedController {

    public enum TypeOfMotor {

        TALON_SRX, PWM_TALON_SRX, VICTOR, SPARK, CAN_SPARK_BRUSHLESS, CAN_SPARK_BRUSHED, JAGUAR, VICTOR_SPX,
        PWM_VICTOR_SPX

    }

    

    private MotorController frcMotor;

    private TypeOfMotor motorToUse;

    public TypeOfMotor getType() {
        return motorToUse;
    }

    boolean inverted;

    public TecbotSpeedController(int port, TypeOfMotor m) {

        motorToUse = m;

        switch (motorToUse) {

            

            case PWM_TALON_SRX:

                frcMotor = new PWMTalonSRX(port);

                break;

            case VICTOR:

                frcMotor = new Victor(port);

                break;

            case SPARK:

                frcMotor = new Spark(port);

                break;

            case JAGUAR:

                frcMotor = new Jaguar(port);

                break;

            

            case PWM_VICTOR_SPX:

                frcMotor = new PWMVictorSPX(port);

                break;
            case CAN_SPARK_BRUSHLESS:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

                getCANSparkMax().restoreFactoryDefaults();

                // TODO check if this kills the code ?

                break;
            case CAN_SPARK_BRUSHED:

                frcMotor = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushed);

                break;
            default:

                DriverStation.reportError("That type of motor doesn't exist!", true);

        }

    }

    public void set(double speed) {
        speed *= inverted ? -1 : 1;

       
        if (frcMotor != null)
            frcMotor.set(speed);

        
    }

    public double getEncPosition() {

        switch (motorToUse) {
            
            case CAN_SPARK_BRUSHLESS:
                return ((CANSparkMax) frcMotor).getEncoder().getPosition();

            default:
                DriverStation.reportWarning("That is not a Talon SRX nor a Spark Max!", true);
                return 0;
        }

    }

    public void stopMotor() {

        if (frcMotor != null)
            frcMotor.stopMotor();
       

    }

    public double get() {

        
        if (frcMotor != null)
            return frcMotor.get();
        else
            DriverStation.reportError("Null motor", true);
        return 0;

    }

    public void setEncoderPosition(int value) {

        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS) {
            ((CANSparkMax) frcMotor).getEncoder().setPosition(0);
            return;
        }
        DriverStation.reportWarning("Not a talonSRX, sensor position not updated", false);

    }

    
    public CANSparkMax getCANSparkMax() {
        if (motorToUse == TypeOfMotor.CAN_SPARK_BRUSHLESS || motorToUse == TypeOfMotor.CAN_SPARK_BRUSHED) {
            return (CANSparkMax) frcMotor;
        }
        return null;
    }

    /*
     * WARNING: this will only work with TALON SRX and Spark Max
     *
     */
    public void setBrakeMode(boolean doBrake) {

        if (frcMotor != null)
            ((CANSparkMax) frcMotor).setIdleMode(doBrake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);

    }

    public void setInverted(boolean i) {
        inverted = i;
    }

    public boolean isInverted() {
        return inverted;
    }

}