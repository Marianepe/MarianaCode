// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;
import frc.robot.resources.TecbotController.ButtonType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/** Add your docs here. */
public class OI {
   public static OI instance;
 private TecbotController Driver1;
 private TecbotController Driver2;
 

    public OI(){
      Driver1 = new TecbotController(RobotMap.pilotPort, TecbotConstants.CONTROLLER_TYPE_PILOT);
      Driver2 = new TecbotController(RobotMap.copilotPort, TecbotConstants.CONTROLLER_TYPE_COPILOT);

   }

   public void configureButtonBindings(){
      
     /* Regina.whenPressed(TecbotController.ButtonType.A, new OnArmT());
      Regina.whenPressed(TecbotController.ButtonType.B, new OffArm());
      Regina.whenPressed(TecbotController.ButtonType.X, new ChangeToSpeed());
      Regina.whenPressed(TecbotController.ButtonType.Y, new ChangeToTorque());
*/
    /*  Regina.whenPressed(TecbotController.ButtonType.A, new IntakeOn());   //este abre
      Regina.whenPressed(TecbotController.ButtonType.B, new IntakeOff());   // este cierra
      Regina.whenPressed(TecbotController.ButtonType.X, new ExtendArm());   //extiende
      Regina.whenPressed(TecbotController.ButtonType.Y, new RetractArm());  // retrae

      Regina.whenPressed(TecbotController.ButtonType.LB, new ChangeToSpeed()); //Left Bumper
      Regina.whenPressed(TecbotController.ButtonType.RB, new ChangeToTorque()); //Right bumper
      
      

      Mario.whenPressed(TecbotController.ButtonType.A, new OnArmT()); // x   ////  este baja
      Mario.whenPressed(TecbotController.ButtonType.B, new OffArm()); // o   //// este sube
     */
    // Mario.whileHeld(TecbotController.ButtonType.X, new ChangeToSpeed()); // []
     // Mario.whileHeld(TecbotController.ButtonType.Y, new ChangeToTorque()); // A
   }

   public static OI getInstance() {
      if (instance == null)
          instance = new OI();

      return instance;
  }

   public TecbotController getPilot(){
      return Driver1;
   }

   public TecbotController getCopilot(){
      return Driver2;
   }

}