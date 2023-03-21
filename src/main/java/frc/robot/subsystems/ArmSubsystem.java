// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  // Tilt Stuff
  private final VictorSPX m_TiltMotor = new VictorSPX(Constants.Arm.Tilt.k_MotorID);
  DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(Constants.Arm.Tilt.Encoder.k_ID);
  private double wantedTiltPosition = Constants.Arm.Tilt.Encoder.k_Min;
  PIDController tiltPID = new PIDController(8, 4, 0);
  // 8 4 0
  // 6 0 2

  // Extension Stuff
  private final TalonSRX m_ExtensionMotor = new TalonSRX(Constants.Arm.Extension.k_MotorID);
  DigitalInput extensionSwitchLeft = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_LeftID);
  DigitalInput extensionSwitchRight = new DigitalInput(Constants.Arm.Extension.LimitSwitch.k_RightID);
  boolean hasExtensionZeroed = false;

  public ArmSubsystem() {
    m_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_ExtensionMotor.setInverted(InvertType.InvertMotorOutput);
    tiltEncoder.setPositionOffset(Constants.Arm.Tilt.Encoder.k_Offset);
  }

  public void Extend() {
    // Check if the extension has done a zeroing procedure yet. If not, don't run
    // the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
    } else if (-m_ExtensionMotor.getSelectedSensorPosition() < Constants.Arm.Extension.Encoder.k_Max) {
      if ((-m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Max) > 0.75) {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_LimitSpeed);
      } else {
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      StopExtension();
    }
  }

  public void Retract() {
    System.out.println("extensionSwitchLeft 22342342");
    // Check if the extension has done a zeroing procedure yet. If not, don't run
    // the motor too fast.
    if (!hasExtensionZeroed) {
      m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
    } else if (-m_ExtensionMotor.getSelectedSensorPosition() > Constants.Arm.Extension.Encoder.k_Min
        && (!extensionSwitchLeft.get() && // Do NOT move the arm in if one of our limit switches is active.
            !extensionSwitchRight.get())) {
      if ((-m_ExtensionMotor.getSelectedSensorPosition() / Constants.Arm.Extension.Encoder.k_Min) < 0.25) {
        System.out.println("extensionSwitchLeft 2");
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_LimitSpeed);
      } else {
        System.out.println("extensionSwitchLeft");
        m_ExtensionMotor.set(TalonSRXControlMode.PercentOutput, Constants.Arm.Extension.k_NormalSpeed);
      }
    } else {
      System.out.println("extensionSwitchLeft 3");
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

  public boolean GetHasExtensionZeroed() {
    return hasExtensionZeroed;
  }

  public double GetArmExtensionEncoderValue() {
    return -m_ExtensionMotor.getSelectedSensorPosition();
  }

  public double GetTiltEncoderValue() {
    return -tiltEncoder.get();
  };

  public double GetRawTiltEncoderPosition() {
    return tiltEncoder.getAbsolutePosition();
  }

  public boolean GetTiltEncoderConnected() {
    return tiltEncoder.isConnected();
  };

  public double GetWantedArmTiltPosition() {
    return wantedTiltPosition;
  };

  public void SetTiltPosition(double tiltPos) {
    wantedTiltPosition = tiltPos;
    if (wantedTiltPosition > Constants.Arm.Tilt.Encoder.k_Max)
      wantedTiltPosition = Constants.Arm.Tilt.Encoder.k_Max;

    if (wantedTiltPosition < Constants.Arm.Tilt.Encoder.k_Min)
      wantedTiltPosition = Constants.Arm.Tilt.Encoder.k_Min;
  }

  public CommandBase TiltUp() {
    return runOnce(
        () -> {
          wantedTiltPosition += 0.02;
          if (wantedTiltPosition > Constants.Arm.Tilt.Encoder.k_Max)
            wantedTiltPosition = Constants.Arm.Tilt.Encoder.k_Max;
        });
  }

  public CommandBase TiltDown() {
    return runOnce(
        () -> {
          wantedTiltPosition -= 0.02;
          if (wantedTiltPosition < Constants.Arm.Tilt.Encoder.k_Min)
            wantedTiltPosition = Constants.Arm.Tilt.Encoder.k_Min;
        });
  }


  @Override
  public void periodic() {
    double pidOutput = tiltPID.calculate(GetTiltEncoderValue(), wantedTiltPosition);
    if (hasExtensionZeroed) {
      m_TiltMotor.set(VictorSPXControlMode.PercentOutput, -MathUtil.clamp(pidOutput, 0, 0.7));
    }

    if (extensionSwitchLeft.get() && extensionSwitchRight.get()) {
      m_ExtensionMotor.setSelectedSensorPosition(0);
      hasExtensionZeroed = true;
    }
  }
}
