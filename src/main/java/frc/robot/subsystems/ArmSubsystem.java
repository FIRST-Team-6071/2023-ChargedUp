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
  PIDController tiltPID = new PIDController(10, 5, 0);
  // 8 4 0
  // 6 0 2

  public ArmSubsystem() {
    tiltEncoder.setPositionOffset(Constants.Arm.Tilt.Encoder.k_Offset);
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
    m_TiltMotor.set(VictorSPXControlMode.PercentOutput, -MathUtil.clamp(pidOutput, 0, 0.8));
  }
}
