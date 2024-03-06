// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HookSub;

public class HookingCmd extends Command {
  private final HookSub m_hookMotors;
  private final XboxController m_controller = new XboxController(0);
  private final PS5Controller ps5Controller = new PS5Controller(1);
  /** Creates a new HookingCmd. */
  public HookingCmd(HookSub hookMotor) {
    this.m_hookMotors = hookMotor;
    addRequirements(hookMotor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_hookMotors.hookWDouble(m_controller.getRightY());
    m_hookMotors.hookWDouble(ps5Controller.getRightY());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
