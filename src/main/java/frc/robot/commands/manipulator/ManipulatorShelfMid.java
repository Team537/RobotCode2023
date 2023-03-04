// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arminout.ArmInOutShelfMid;
import frc.robot.commands.armpivot.ArmPivotShelfMid;
import frc.robot.commands.led.LedShelf;
import frc.robot.commands.wrist.WristShelfMid;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.manipulator.ArmInOut;
import frc.robot.subsystems.manipulator.ArmPivot;
import frc.robot.subsystems.manipulator.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManipulatorShelfMid extends SequentialCommandGroup {



  
  /** Creates a new ManipulatorHighGoal. */
  public ManipulatorShelfMid(ArmPivot m_ArmPivot, ArmInOut m_ArmInOut, Wrist m_Wrist, LED m_LED) {

    
     
   
    
     
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new ParallelCommandGroup(
        new LedShelf(m_LED),
        new ParallelCommandGroup(
          new ArmPivotShelfMid(m_ArmPivot),  
          new WristShelfMid(m_Wrist)),
        new WaitCommand(1), 
        new ArmInOutShelfMid(m_ArmInOut)
      )
    
    );
  }
}
