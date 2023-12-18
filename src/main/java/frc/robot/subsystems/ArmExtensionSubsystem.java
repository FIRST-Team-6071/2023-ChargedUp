// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtensionSubsystem extends SubsystemBase {
  // Extension Stuff
  private final TalonSRX m_ExtensionMotor = new TalonSRX(Constants.Arm.Extension.k_MotorID);
  DigitalInput extensionSwitchLeft = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_LeftID);
  DigitalInput extensionSwitchRight = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_RightID);
  boolean hasExtensionZeroed = false;

  /** Creates a new ArmExtensionSubsystem. */
  public ArmExtensionSubsystem() {
    m_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_ExtensionMotor.setInverted(InvertType.InvertMotorOutput);
  }

  public void Extend() {
    // Check if the extension has done a zeroing procedure yet. If not, don't run
    // the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
    } else if (-m_ExtensionMotor.getSelectedSensorPosition() < Constants.Arm.Extension.Encoder.k_Max) {
      if ((-m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Max) > 0.75) {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
      } else {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      StopExtension();
    }
  }

  public void Retract() {
    // Check if the extension has done a zeroing procedure yet. If not, don't run
    // the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
    } else if (-m_ExtensionMotor.getSelectedSensorPosition() > Constants.Arm.Extension.Encoder.k_Min
        && (!extensionSwitchLeft.get())) {
      if ((-m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Max) < 0.25) {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
      } else {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      StopExtension();
    }

    if (extensionSwitchLeft.get()) {
      m_ExtensionMotor.setSelectedSensorPosition(0);
      hasExtensionZeroed = true;
    }
  }

  public void StopExtension() {
    m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean GetHasExtensionZeroed() {
    return hasExtensionZeroed;
  }

  public double GetArmExtensionEncoderValue() {
    return -m_ExtensionMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((extensionSwitchLeft.get()) && !hasExtensionZeroed) {
      m_ExtensionMotor.setSelectedSensorPosition(0);
      hasExtensionZeroed = true;
    }
  }
}
