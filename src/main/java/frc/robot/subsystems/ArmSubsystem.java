// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Tilt Stuff
  private final VictorSPX m_TiltMotor = new VictorSPX(Constants.Arm.Tilt.k_MotorID);
  // Encoder tiltEncoder = new Encoder(Constants.MaxETilt.KTMax, Constants.MinETilt.KTMin);

  // Extension Stuff
  private final TalonSRX m_ExtensionMotor = new TalonSRX(Constants.Arm.Extension.k_MotorID);
  DigitalInput extensionSwitchLeft = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_LeftID);
  DigitalInput extensionSwitchRight = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_RightID);
  boolean hasExtensionZeroed = false;

  public ArmSubsystem() {
    m_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_ExtensionMotor.setInverted(InvertType.InvertMotorOutput);
  }

  public void Extend() {
    // Check if the extension has done a zeroing procedure yet. If not, don't run the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
    } else if (m_ExtensionMotor.getSelectedSensorPosition() < Constants.Arm.Extension.Encoder.k_Max) {
      if ((m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Max) > 0.75) {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
      } else {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      StopExtension();
    }
  }

  public void Retract() {
    // Check if the extension has done a zeroing procedure yet. If not, don't run the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
    } else if (
      m_ExtensionMotor.getSelectedSensorPosition() > Constants.Arm.Extension.Encoder.k_Min && (
        !extensionSwitchLeft.get() && // Do NOT move the arm in if one of our limit switches is active.
        !extensionSwitchRight.get() 
      )
    ) {
      if ((m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Min) < 0.25) {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
      } else {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      StopExtension();
    }

    if (extensionSwitchLeft.get() && extensionSwitchRight.get()) {
      m_ExtensionMotor.setSelectedSensorPosition(0);
      hasExtensionZeroed = true;
    }
  }

  public void StopExtension() {
    m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean GetHasExtensionZeroed() { return hasExtensionZeroed; }
  public double GetArmExtensionEncoderValue() { return m_ExtensionMotor.getSelectedSensorPosition(); }
  // public void up() {
  //   m_ExtendedArm.set(ControlMode.PercentOutput, Constants.ArmExtendSpeed.ExtendSpeed);
  //   m_ExtendedArm.set(Constants.ArmExtendSpeed.ExtendSpeed);
  // }

  // public void down() {
  //   m_ExtendedArm.set(-Constants.ArmExtendSpeed.ExtendSpeed);
  // }

  // public void extend() {
  //   m_TiltArm.set(Constants.ArmDirectionSpeed.DSpeed);
  // }

  // public void retract() {
  //   m_TiltArm.set(Constants.ArmDirectionSpeed.DSpeed);
  // }

  // public void TiltEncoder(int KTMax, int KTMin) {
  //   if (KTMax > 1000) {
  //     tiltEncoder.getStopped();
  //   }

  //   if (KTMin == 0) {
  //     // whatever encoder go is in java
  //     tiltEncoder.get();
  //   }
  // }

  // public void extendEncoder(int KEMax, int KEMin) {
  //   if (KEMax > 1000) {
  //     extendEncoder.getStopped();
  //   }

  //   if (KEMin == 0) {
  //     // whatever encoder go is in java
  //     extendEncoder.get();
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
