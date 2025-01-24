package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsytem;

public class AimCommand extends Command 
{
    private LimelightSubsytem limeLight;
    private CommandSwerveDrivetrain drivetrain;

    private static final PIDController rotationalPidController = new PIDController(0.05, 0, 0, 0.5);

    private static final PIDController xPidController = new PIDController(0.3, 0, 0, 0.2);
    private static final PIDController yPidController = new PIDController(0.3, 0, 0, 0.5);

    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric();
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    //#region attempt1

    /*

    private final LimelightHelpers visionBase;
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

    public AimCommand(LimelightHelpers visionBase, SwerveDrivetrain drivetrain, double newHoldDistance, int newPipelineID)
    {
        m_visionBase = visionBase;
        m_drivetrain = drivetrain;
        holdDistance = newHoldDistance;
        pipelineID = newPipelineID;

        m_allignRequest = new LegacySwerveRequest.FieldCentric().withDeadband(TunerConstants.kSpeedAt12Volts.magnitude() * 0.1).withRotationalDeadband(0.1);
        aimController = new ProfiledPIDController(aimP, aimI, aimD, new TrapezoidProfile.Constraints(maxAimVelocity, maxAimAccel));
        aimController.enableContinuousInput(-Math.PI, Math.PI);

        rangeController = new ProfiledPIDController(rangeP, rangeI, rangeD, new TrapezoidProfile.Constraints(maxRangeVelocity, maxRangeAccel));

        //addRequirements(drivetrain, visionBase);
    }

    @Override public void initialize()
    {
        LimelightHelpers.setPipelineIndex(limeLightName, pipelineID);
        aimController.reset(0);

        aimController.setGoal(0);
        rangeController.setGoal(holdDistance);
    }

    @Override public void execute()
    {
        
    }
    */
    //#endregion

    //#region Description

    /**Aim Command, the commander of aim, the master of aim, the almighty aimster, the AIM OVERLORD, the aimest one, 
     * the aim side of the force, one aim to rule them all, one aim to find them, one aim to bring them all and in the
     *  darkness bind them; in the land of limelight where the lime lights shine; throughout districs and states I alone 
     * am the aimest one. */

     //#endregion
    public AimCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsytem limelight) 
    {
        this.drivetrain = drivetrain;
        this.limeLight = limelight;
    }
    
    @Override public void initialize() 
    {

    }

    @Override public void execute() 
    {
        RawFiducial fiducial;

        try 
        {
            fiducial = limeLight.getFiducialWithId(4);

            final double rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0) * TunerConstants.MaxAngularRate * 0.2;
            final double velocityX = xPidController.calculate(fiducial.distToRobot, 2.25) * TunerConstants.MaxSpeed * -0.5;

            if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) 
            {
                this.end(true);
            }

            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalPidController", rotationalRate);
            SmartDashboard.putNumber("xPidController", velocityX);
            drivetrain.setControl(alignRequest.withRotationalRate(rotationalRate).withVelocityX(velocityX));

        } 
        catch (LimelightSubsytem.NoSuchTargetException nste) 
        {

        }
  }

  /** Only returns false right now for some reason */
  @Override public boolean isFinished() 
  {
    return false;
  }

  @Override public void end(boolean interrupted) 
  {
    Supplier<SwerveRequest> swerveSupplier = () -> idleRequest;
    drivetrain.applyRequest(swerveSupplier);
  }
}
