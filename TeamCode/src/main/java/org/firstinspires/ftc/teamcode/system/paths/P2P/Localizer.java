package org.firstinspires.ftc.teamcode.system.paths.P2P;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.paths.P2P.C.TwoTrackingWheelLocalizer;

public class Localizer
{
    public static boolean ENABLED = true;
    private Pose pose;
    private com.acmerobotics.roadrunner.localization.Localizer localizer;
    public Localizer(HardwareMap hardwareMap, Pose startingPose, LinearOpMode opMode)
    {
        this.pose = startingPose;
        this.localizer = new TwoTrackingWheelLocalizer(hardwareMap, new ImuThread(hardwareMap), opMode);
        localizer.setPoseEstimate(startingPose.toPose2d());

    }
    public Localizer(HardwareMap hardwareMap, Pose2d startingPose, LinearOpMode opMode)
    {
        this.pose = new Pose(startingPose);
        this.localizer = new TwoTrackingWheelLocalizer(hardwareMap, new ImuThread(hardwareMap), opMode);
        localizer.setPoseEstimate(startingPose);

    }

    public Localizer(HardwareMap hardwareMap, LinearOpMode opMode)
    {
        this.pose = new Pose();
        this.localizer = new TwoTrackingWheelLocalizer(hardwareMap, new ImuThread(hardwareMap), opMode);
        localizer.setPoseEstimate(pose.toPose2d());
    }

    public Localizer(GeneralHardware hardware)
    {
        // implement this
    }
    @Deprecated
    public Pose getPredictedPose()
    {
        return new Pose();
    }

    public void setPose(Pose pose) {
        localizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()));
        this.pose = pose;
    }

    public Pose getPoseEstimate() {
        return pose;
    }

    public Pose getPredictedPoseEstimate(){
        return new Pose(pose.getX() + glideDelta.getX(), pose.getY() + glideDelta.getY(), pose.getHeading());
    }

    public double getHeading(){
        return pose.getHeading();
    }

    private Vector velocity = new Vector();
    public Vector driveTrainVelocity = new Vector();
    public Vector glideDelta = new Vector();

    public static double filterParameter = 0.8;
    private final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0),
            yVelocityFilter = new LowPassFilter(filterParameter, 0);

    public static double xDeceleration = 43, yDeceleration = 73;  //= 55.1, yDeceleration = 2.97;//29.7; // 100 , 150,  50.87

    public Vector getVelocity(){
        return velocity;
    }
    public Vector getGlideDelta() {return glideDelta;}

    public void update() {
        if(!ENABLED) return;

        localizer.update();
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        velocity = new Vector(xVelocityFilter.getValue(localizer.getPoseVelocity().getX()), yVelocityFilter.getValue(localizer.getPoseVelocity().getY()));
        driveTrainVelocity = Vector.rotateBy(velocity, 0);
        Vector predictedGlideVector = new Vector(Math.signum(driveTrainVelocity.getX()) * Math.pow(driveTrainVelocity.getX(), 2) / (2.0 * xDeceleration),

                Math.signum(driveTrainVelocity.getY()) * Math.pow(driveTrainVelocity.getY(), 2) / (2.0 * yDeceleration));
        glideDelta = predictedGlideVector.rotated(-pose.getHeading());
    }



}
