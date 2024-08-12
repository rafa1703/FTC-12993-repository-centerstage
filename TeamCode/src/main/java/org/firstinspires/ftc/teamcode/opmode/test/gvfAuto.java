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
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.firstinspires.ftc.teamcode.system.paths.splines.BelzierCurve;
import org.firstinspires.ftc.teamcode.system.paths.splines.BelzierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.system.paths.splines.GVFLogic;
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
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DriveBase driveBase = new DriveBase(telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.initIntake(hardwareMap);
        intakeSubsystem.intakeHardwareSetup();
        intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        driveBase.setUpFloat();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 100);

        drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.Vector, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(-60, -24, Math.toRadians(180)), this));

        BelzierCurve curve0 = new BelzierCurve(new Point[]{
                new Point(-36, -64),
                new Point(-60, -24),
        });
        BelzierCurve curve1 = new BelzierCurve(new Point[]{
                new Point(-60, -24),
                new Point(-50, 0),
                new Point(-36, 0)
        });

        BelzierCurve curve2 = new BelzierCurve(new Point[]{
                new Point(-36, 0),
                new Point(0, 0),
        });
        BelzierCurve curve3 = new BelzierCurve(new Point[]{
                new Point(0, 0),
                new Point(24, 0),
                new Point(24, -24)
        });
        Trajectory trajectory = new TrajectoryBuilder(new BelzierCurveTrajectorySegment(curve1))
                .addSegment(new BelzierCurveTrajectorySegment(curve2))
                .addSegment(new BelzierCurveTrajectorySegment(curve3))
                .build();

        //drive.setSpeed(1);
        ArrayList<Point> fullCurve = trajectory.getFullCurve();
        waitForStart();

        while(opModeIsActive())
        {
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
            //drive.followTrajectory(trajectory);
            drive.followTrajectoryTangentially(trajectory, true);
            drive.update();


            packet = new TelemetryPacket();
            for (Point point: fullCurve)
            {
                //telemetry.addData("POINT", point);
                packet.fieldOverlay().setFill("black").fillCircle(point.x, point.y, 1);

            }
            packet.fieldOverlay().setFill("Red").fillCircle(drive.getLocalizer().getPoseEstimate().getX(),drive.getLocalizer().getPoseEstimate().getY() , 2);
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
