package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class AimCommand extends Command 
{

    /*
    private final LimelightHelpers m_visionBase;
    private final SwerveDrivetrain m_drivetrain;

    private final double holdDistance;
    private final int pipelineID;

    private final FieldCentric m_allignRequest;
    private final ProfiledPIDController aimController;
    private final ProfiledPIDController rangeController;

    private final double aimP = 0.1;
    private final double aimI = 0.01;
    private final double aimD = 0.05;

    private final double rangeP = 0.1;
    private final double rangeI = 0.01;
    private final double rangeD = 0.05;

    private final double maxAimVelocity = 1.5 * Math.PI;
    private final double maxAimAccel = Math.PI / 2;

    private final double maxRangeVelocity = 1.5 * Math.PI;
    private final double maxRangeAccel = Math.PI / 2;

    private final String limeLightName = "limelight"; 

    public AimCommand(Subsystem visionBase, SwerveDrivetrain drivetrain, double newHoldDistance, int newPipelineID)
    {
        m_visionBase = visionBase;
        m_drivetrain = drivetrain;
        holdDistance = newHoldDistance;
        pipelineID = newPipelineID;

        m_allignRequest = new LegacySwerveRequest.FieldCentric().withDeadband(TunerConstants.kSpeedAt12Volts.magnitude() * 0.1).withRotationalDeadband(0.1);
        aimController = new ProfiledPIDController(aimP, aimI, aimD, new TrapezoidProfile.Constraints(maxAimVelocity, maxAimAccel));
        aimController.enableContinuousInput(-Math.PI, Math.PI);

        rangeController = new ProfiledPIDController(rangeP, rangeI, rangeD, new TrapezoidProfile.Constraints(maxRangeVelocity, maxRangeAccel));

        addRequirements(visionBase);
    }

    @Override public void initialize()
    {
        LimelightHelpers.setPipelineIndex(limeLightName, pipelineID);
        aimController.reset(0);

        aimController.setGoal(0);
        rangeController.setGoal(holdDistance);
    }

    */

    /**runs every 0.02 seconds */
    @Override public void execute()
    {
        
    }
}
