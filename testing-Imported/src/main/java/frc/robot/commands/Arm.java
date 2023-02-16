package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends CommandBase{
    private ArmSubsystem Armm;
    private BooleanSupplier Intake;
    private BooleanSupplier Low;
    private BooleanSupplier Medium;
    private BooleanSupplier High;
    private BooleanSupplier Retract;
    //Constructor for command
    public Arm (ArmSubsystem Armm, BooleanSupplier Intake, BooleanSupplier Low, BooleanSupplier Medium, BooleanSupplier High, BooleanSupplier Retract){
        this.Armm = Armm;
        addRequirements(Armm);
        this.Intake = Intake;
        this.Low = Low;
        this.Medium = Medium;
        this.High = High;
        this.Retract = Retract;

    }
    public void execute(){
        Armm.setPoint(Intake.getAsBoolean(), Low.getAsBoolean(), Medium.getAsBoolean(), High.getAsBoolean(), Retract.getAsBoolean());
        if (Intake.getAsBoolean() || Low.getAsBoolean() || Medium.getAsBoolean() || High.getAsBoolean() || Retract.getAsBoolean()){
            Armm.moveToAngle();
        }
    }

}
