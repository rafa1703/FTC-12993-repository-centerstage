package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "P2P Tune")
public class p2pTune extends LinearOpMode
{
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
                MecanumDrive.RunMode.P2P, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(0)), this));

        drive.setTargetPose(new Pose(15, 15, Math.toRadians(0)));
        waitForStart();
        while (opModeIsActive())
        {

            drive.update();
            telemetry.addData("Pose estimate", drive.getLocalizer().getPoseEstimate());
            telemetry.addData("Predicted pose estimate", drive.getLocalizer().getPredictedPoseEstimate());
            telemetry.addData("Heading diff", Math.toDegrees(drive.getTargetPose().getHeading() - drive.getLocalizer().getPoseEstimate().getHeading()));
            telemetry.addData("Reached", drive.reachedTarget(0.5));
            telemetry.update();
        }
    }
}
