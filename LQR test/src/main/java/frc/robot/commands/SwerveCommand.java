package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.oi.OI;
import frc.robot.subsystems.SwerveModule;

public class SwerveCommand extends Command{
    public final SwerveModule swerveModule;
    public final OI oi;
    public SwerveCommand(SwerveModule swerveModule, OI oi){
        this.swerveModule = swerveModule;
        this.oi =  oi;
        addRequirements(swerveModule);
    }

    public void initialize(){
        swerveModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
    }

    public void execute(){
        swerveModule.setState(new SwerveModuleState(0, new Rotation2d(oi.driverController().getAxis(OI.Axes.LEFT_STICK_X))));
    }

    public void end(boolean isFinished){
        swerveModule.setState(new SwerveModuleState(0, new Rotation2d(0)));
    }

    public boolean isFinished(){
        return false;
    }
}
