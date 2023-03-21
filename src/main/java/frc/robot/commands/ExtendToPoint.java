// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendToPoint extends PIDCommand {
  ArmSubsystem m_Arm;
  double extPoint;
  /** Creates a new ExtendToPoint. */
  public ExtendToPoint(double p_extPoint, ArmSubsystem p_ArmSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(6, 2, 0),
        // This should return the measurement
        () -> p_ArmSubsystem.GetArmExtensionEncoderValue(),
        // This should return the setpoint (can also be a constant)
        () -> p_extPoint,
        // This uses the output
        output -> {
          if (output < 0) p_ArmSubsystem.Retract();
          else if (output > 0) p_ArmSubsystem.Extend();
          else p_ArmSubsystem.StopExtension();
          // Use the output here
        });
    
    extPoint = p_extPoint;
    m_Arm = p_ArmSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_ArmSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.GetArmExtensionEncoderValue() > extPoint - 1000 && m_Arm.GetArmExtensionEncoderValue() < extPoint + 1000) {
      return true;
    }

    return false;
  }
}
