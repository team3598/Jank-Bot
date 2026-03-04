package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;


public class PoseSubsystem extends SubsystemBase {

    public static double timeMS = Timer.getFPGATimestamp();
    final Field2d field = new Field2d();
    private final List<String> limelightNames = List.of("limelight-fleft", "limelight-fright");//, "limelight-fright"); //include limelight-fright later on when you figure out how to get avgs between two limelights
    private int loopCounter = 0;
    private final Vector<N3> defaultStdDevs = VecBuilder.fill(0.1, 0.1, 0.4);
    private final CommandSwerveDrivetrain drivetrain;
    
    public PoseSubsystem(CommandSwerveDrivetrain drivetrain) { //constructor
        this.drivetrain = drivetrain;        
        SmartDashboard.putData(field); 
    }
    
    private Vector<N3> calculateStdDev(double distance) {
        if (distance < 1.0) return defaultStdDevs; 

        double xyUncertainty = 0.1 * Math.pow(distance, 4);
        double thetaUncertainty = 0.4 * Math.pow(distance, 4);
        
        
        if (xyUncertainty < 0.1) xyUncertainty = 0.1;
        if (thetaUncertainty < 999999999.0) thetaUncertainty = 9999999999999.0;

        return VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 
    }

    private void updateVision(String name) {
        if (!LimelightHelpers.getTV(name)) {return;} //if limelight not exist, then dont even bother running the rest
    
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            
        if (mt2 == null || mt2.tagCount == 0 || mt2.avgTagDist > 4.0) //|| 
            {return;}
    
        LimelightHelpers.SetRobotOrientation(name, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0); //gives MT2 the current rotation of the bot
    
        Vector<N3> stdDevs = calculateStdDev(mt2.avgTagDist);
    
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);
    }
    
    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    public double getDistToTarget(Translation2d targetPosition) {
        return drivetrain.getState().Pose.getTranslation().getDistance(targetPosition);
    }

    public void printCurrentPose(){
        System.out.println(drivetrain.getState().Pose);
    }
    
    @Override
    public void periodic() {
        timeMS = Timer.getFPGATimestamp(); //20 ms
        String activeCamera = limelightNames.get(loopCounter % limelightNames.size()); //i do this because it is really taxing on the RoboRio to calculate everything and process two limelights at the same time. by doing this, it's switching back and forth between the two which should give me some less lag
        loopCounter++;
        updateVision(activeCamera);
        //printCurrentPose();
        if (loopCounter % 2 == 0) {
            field.setRobotPose(drivetrain.getState().Pose);
        }
    }
}
