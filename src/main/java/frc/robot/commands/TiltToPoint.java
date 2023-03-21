// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class TiltToPoint extends CommandBase {
  private ArmSubsystem m_Arm;
  private double tiltPos;

  /** Creates a new TiltToPoint. */
  public TiltToPoint(double p_tiltPos, ArmSubsystem p_ArmSubsystem) {
    m_Arm = p_ArmSubsystem;
    tiltPos = p_tiltPos;

    addRequirements(p_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.SetTiltPosition(tiltPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.GetTiltEncoderValue() > tiltPos - 0.010 && m_Arm.GetTiltEncoderValue() < tiltPos + 0.010) {
      return true;
    }

    return false;
  }
}
