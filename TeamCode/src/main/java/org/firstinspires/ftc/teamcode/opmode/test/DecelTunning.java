package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;


@Config
@TeleOp(name = "Deceleration Tuner", group="Test")
public class DecelTunning extends LinearOpMode {

    FtcDashboard dash;

    GeneralHardware hardware;

    MecanumDrive drive;

    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 1;
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        //hardware = new GeneralHardware(hardwareMap, GeneralHardware.Color.Blue);

        /*drive = new MecanumDrive(hardwareMap, MecanumDrive.RunMode.Vector, false);
        hardware.startThreads(this);*/
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
        drive.setLocalizer(new Localizer(hardwareMap, new Pose(0, 0, Math.toRadians(0)), this));

        waitForStart();


        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {

            //hardware.update();

            switch (step){
                case 0:
                    if(timer.seconds() <= accelerationTime) {
                        drive.setTargetVector(new Vector(1,0));
                    }
                    else{
                        step++;
                        timer.reset();
                        velocityAtStop = drive.getLocalizer().getVelocity().getX();
                        drive.setTargetVector(new Vector());
                    }
                    break;
                case 1:
                    if(drive.getLocalizer().getVelocity().getMagnitude() <= 0.1)
                    {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = velocityAtStop / deltaTime;
                        stopped = drive.stopped();
                    }
                    break;
            }
            // 2.62 y decel, 87.68 x decel

            drive.update();

            telemetry.addData("pose", drive.getLocalizer().getPoseEstimate());
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Deceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", drive.getLocalizer().getVelocity().getX());
            telemetry.addData("Velocity y", drive.getLocalizer().getVelocity().getY());
            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());
            loopTimer.reset();

            telemetry.update();
        }
    }
}