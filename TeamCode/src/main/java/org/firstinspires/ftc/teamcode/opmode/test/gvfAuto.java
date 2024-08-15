package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurve;
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.system.paths.splines.Trajectory;
import org.firstinspires.ftc.teamcode.system.paths.splines.TrajectoryBuilder;
import org.opencv.core.Point;

import java.util.ArrayList;

@Autonomous(name = "GVFAuto")
public class gvfAuto extends LinearOpMode
{

    MecanumDrive drive;
    TelemetryPacket packet;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean marker = false;
    ArrayList<Point> pathTraveled = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DriveBase driveBase = new DriveBase(telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        driveBase.setUpFloat();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 100);

        drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.Vector, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(-36, 0, Math.toRadians(180)), this));

        BezierCurve curve0 = new BezierCurve(new Point[]{
                new Point(-36, -64),
                new Point(-60, -24),
        });
        BezierCurve curve1 = new BezierCurve(new Point[]{
                new Point(-60, -24),
                new Point(-50, 0),
                new Point(-36, 0)
        });

        BezierCurve curve2 = new BezierCurve(new Point[]{
                new Point(-36, 0),
                new Point(0, 0),
        });
        BezierCurve curve3 = new BezierCurve(new Point[]{
                new Point(0, 0),
                new Point(24, 0),
                new Point(24, -24)
        });
        BezierCurve lineToSplineHeading = new BezierCurve(new Point[]{
                new Point(0, 0),
                new Point(15, 15),
        });
        Trajectory trajectory = new TrajectoryBuilder(drive.getLocalizer().getPoseEstimate())
                .addSegment(new BezierCurveTrajectorySegment(curve2))
                .addSegment(new BezierCurveTrajectorySegment(curve3, 0.7))
                //.addSegment(new BezierCurveTrajectorySegment(curve2))
                //.addSegment(new BezierCurveTrajectorySegment(curve3))
                //.addSegment(new BezierCurveTrajectorySegment(curve3))
//                .addSpatialMarker(new Pose(24, -24), () ->
//                {
//                    marker = true;
//                })
                .addFinalPose(new Pose(24, -24, Math.toRadians(180)))
                .build();

        //drive.setSpeed(1);
        ArrayList<Point> fullCurve = trajectory.getFullCurve();
        waitForStart();

        while(opModeIsActive())
        {
            intakeSubsystem.intakeClipServoState(IntakeSubsystem.IntakeClipServoState.HOLDING);
            //drive.followTrajectory(trajectory);
            //drive.followTrajectoryTangentially(trajectory, true);
            drive.followTrajectory(trajectory);
            drive.update();


            packet = new TelemetryPacket();
            for (Point point: fullCurve)
            {
                //telemetry.addData("POINT", point);
                packet.fieldOverlay().setFill("black").fillCircle(point.x, point.y, 1);

            }
            for (Point point : pathTraveled)
            {
                packet.fieldOverlay().setFill("Orange").fillCircle(point.x, point.y, 0.8);
            }
            packet.fieldOverlay().setStroke("Blue").strokeRect(drive.getLocalizer().getPredictedPoseEstimate().getX(), drive.getLocalizer().getPredictedPoseEstimate().getY(), 15, 16);
            packet.fieldOverlay().setStroke("Red").strokeRect(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY(), 15, 16);
            pathTraveled.add(drive.getLocalizer().getPoseEstimate().toPoint());


            //packet.fieldOverlay().setFill("Blue").fillCircle(drive.getLocalizer().getPredictedPoseEstimate().getX(),drive.getLocalizer().getPredictedPoseEstimate().getY() , 2);
            //packet.fieldOverlay().setFill("Red").fillCircle(drive.getLocalizer().getPoseEstimate().getX(),drive.getLocalizer().getPoseEstimate().getY() , 2);

           /* telemetry.addData("Marker", marker);
            telemetry.addData("Pose predinct", drive.getLocalizer().getPredictedPoseEstimate());
            telemetry.addData("Pose", drive.getLocalizer().getPoseEstimate());
            telemetry.update();*/

            //packet.fieldOverlay().setFill("orange").fillRect(pose.getX(), pose.getY(), 1, 4).;
            /*telemetry.addData("Heading", Math.toDegrees(drive.getLocalizer().getHeading()));
            telemetry.addData("HeadingScale", gvfLogic.headingS );
            telemetry.addData("Z vector", Math.toDegrees(gvfLogic.zVector));
            telemetry.addData("looptime");
            telemetry.update();*/
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
