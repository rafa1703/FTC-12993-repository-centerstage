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
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurve;
import org.firstinspires.ftc.teamcode.system.paths.splines.GVFLogic;
import org.opencv.core.Point;

import java.util.ArrayList;
@Autonomous(name = "GVFTest")
public class gvfTest extends LinearOpMode
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
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(-48, 0, Math.toRadians(180)), this));

        BezierCurve curve = new BezierCurve(new Point[]{
                new Point(-48, 0),
                new Point(0, 0),
                new Point(24, 0),
                new Point(24, -24)
        });
        GVFLogic gvfLogic = new GVFLogic();
        gvfLogic.followTangentially = true;
        gvfLogic.reverse = true;
        ArrayList<Point> curvePoints = curve.returnCurve();
        //drive.setSpeed(1);
        waitForStart();

        while(opModeIsActive())
        {
            intakeSubsystem.intakePixelHolderServoState(IntakeSubsystem.IntakePixelHolderServoState.HOLDING);
            Pose pose = drive.getLocalizer().getPredictedPoseEstimate();
            Vector power = gvfLogic.calculate(curve, pose, true); // might want to pass as Pose
            drive.setTargetVector(power);

            drive.update();


            packet = new TelemetryPacket();
            for (Point point: curvePoints)
            {
                //telemetry.addData("POINT", point);
                packet.fieldOverlay().setFill("black").fillCircle(point.x, point.y, 1);

            }
            packet.fieldOverlay().setFill("Red").fillCircle(pose.getX(), pose.getY(), 2);
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
