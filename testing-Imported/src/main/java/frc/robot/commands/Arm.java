package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends CommandBase{
    private ArmSubsystem Armm;
    private DoubleSupplier forwaSupplier;


    //Constructor for command
    public Arm (ArmSubsystem Armm, DoubleSupplier forwaSupplier){
        this.Armm = Armm;
        addRequirements(Armm);
        this.forwaSupplier = forwaSupplier;
    }
    public void execute(){
        ;
    }
}
