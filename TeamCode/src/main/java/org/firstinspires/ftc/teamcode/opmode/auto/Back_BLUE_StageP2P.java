package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.headingPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.poseEstimate;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.xPosition;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.yPosition;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.INTAKE_SLIDE_AUTO_LONG_PRESET;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.Place;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.S;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.angleWrap;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.frontOrBackAuto;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.isArmDown;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.numCycles;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.place;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.teamPropLocation;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.ticksToInchesSlidesMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBase;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Localizer;
import org.firstinspires.ftc.teamcode.system.paths.P2P.MecanumDrive;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurve;
import org.firstinspires.ftc.teamcode.system.paths.splines.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.system.paths.splines.Trajectory;
import org.firstinspires.ftc.teamcode.system.paths.splines.TrajectoryBuilder;
import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name = "Back Blue Stage Auto P2P", group = "Autonomous")
public class Back_BLUE_StageP2P extends LinearOpMode {

    int numCycleForDifferentLane = 0;
    double delayForYellow = 0; // this is in seconds
    double endAngleForStacks = -176.8;
    boolean didWeFuckingRelocalize = false;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry,3);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto,true,3);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,3);
    private double railTarget;

    Trajectory purpleYellowTrajectory = new org.firstinspires.ftc.teamcode.system.paths.splines.TrajectoryBuilder(new Pose(9.9, 59, Math.toRadians(180)))
            .addSegment(
                    new BezierCurveTrajectorySegment(new BezierCurve(new Point[]{
                            new Point(9.9, 59),
                            new Point(36, 29)
                    })))
            .addFinalPose(new Pose(36, 29, Math.toRadians(180)))
            .build();

    Trajectory firstIntakeTrajectory = new org.firstinspires.ftc.teamcode.system.paths.splines.TrajectoryBuilder(purpleYellowTrajectory.getFinalPose())
            .addSegment(
                    new BezierCurveTrajectorySegment(new BezierCurve(new Point[]{
                            new Point(36, 29),
                            new Point(24, 3),
                            new Point(12, 6.4),
                            new Point(0, 6.4),
                            new Point(-12, 6.4),
                            new Point(-28.7, 6.4),

                    })))
            .addFinalPose(new Pose(-28, 6.4, Math.toRadians(180)))
            .build();

    Trajectory depositTrajectory = new TrajectoryBuilder(firstIntakeTrajectory.getFinalPose())
            .addSegment(
                    new BezierCurveTrajectorySegment(new BezierCurve(new Point[]{
                            new Point(-28.7, 6.4),
                            new Point(14.8, 6.4),
                            new Point(20, 6.4),
                            new Point(27, 12),
                    })))
            .addFinalPose(new Pose(27, 12, Math.toRadians(180)))
            .build();

    enum AutoState {
        DELAY,
        PRELOAD_DRIVE,
        PLACE_AND_INTAKE,
        AFTER_PURPLE_DRIVE,
        TRANSFER_PIXEL,
        OUTTAKE_PIXEL,
        DROP,
        GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK,
        AFTER_GRAB_OFF_STACK_TOP,
        OUTTAKE_PIXEL_NO_INTAKE_SLIDES,
        GO_BACK_FOR_YELLOW,
        GO_BACK_FOR_WHITES,
        PARK,
        IDLE,
        DELAY_BACK,
        PRELOAD_DRIVE_BACK,
        PLACE_AND_INTAKE_BACK,
        PRELOAD_DRIVE_CASE_3,
        AFTER_PRELOAD_DRIVE_3
    }

    AutoState currentState;
    MecanumDrive drive;
    TelemetryPacket packet;
    ArrayList<Point> pathTraveled = new ArrayList<>();
    Trajectory trajectoryBeingFollowed = purpleYellowTrajectory;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        frontOrBackAuto = false;
        SetAuto.setBlueAuto();
        auto.setGamepad1(gamepad1);
        if (!frontOrBackAuto){
            isArmDown = true;
        }



        //DriveConstants.MAX_VEL = 60;
        //DriveConstants.MAX_ACCEL = 52;

        if (S == -1){
            auto.autoTrajectories.MiddleLaneYIntake -= 3.9;
        }

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        auto.initAutoHardware(hardwareMap,this);

        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            auto.intializationLoop(!isArmDown);
            auto.setPlaceForBack();
            place = Place.RIGHT;
            if (teamPropLocation == 1){
                telemetry.addLine("Front");
            } else if (teamPropLocation == 2){
                telemetry.addLine("Middle");
            } else if (teamPropLocation == 3){
                telemetry.addLine("Back");
            }
            telemetry.addData("S", S);

        }


        waitForStart();
        if (isStopRequested()) return;

        DriveBase driveBase = new DriveBase(telemetry);
        driveBase.initDrivebase(hardwareMap);
        driveBase.drivebaseSetup();
        driveBase.setUpFloat();

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        TimedSupplier<Double> voltageSupplier = new TimedSupplier<>(voltageSensor::getVoltage, 100);

        drive = new MecanumDrive(
                driveBase.FL, driveBase.FR, driveBase.BL, driveBase.BR,
                MecanumDrive.RunMode.Vector, voltageSupplier);
        drive.setLocalizer(new Localizer(hardwareMap, auto.autoTrajectories.startPoseBack, this));

        auto.afterWaitForStart(!frontOrBackAuto, frontOrBackAuto? auto.autoTrajectories.startPoseFront: auto.autoTrajectories.startPoseBack);
        if (frontOrBackAuto){
            currentState = AutoState.DELAY;
        } else {
            currentState = AutoState.DELAY_BACK;
        }

        // can set drive constraints here
        while (opModeIsActive() && !isStopRequested()) {
            packet = new TelemetryPacket();
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
                module.clearBulkCache();
            }

            auto.mainAutoLoop(
                    currentState == AutoState.OUTTAKE_PIXEL && xPosition > 20,
                    (currentState == AutoState.GRAB_OFF_STACK && xPosition < -25) || currentState == AutoState.AFTER_GRAB_OFF_STACK || currentState == AutoState.PLACE_AND_INTAKE,
                    currentState != AutoState.PRELOAD_DRIVE && currentState != AutoState.OUTTAKE_PIXEL);


            try {
                autoSequence();
            } catch (Exception e){

            }


            loopTime.delta();
            telemetry.addLine();
            //telemetry.addData("numCycles", numCycles);
            //telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);

            telemetry.addData("X OFfset", auto.cameraHardware.ROBOT_X);
            telemetry.addData("Y Offset", auto.cameraHardware.ROBOT_Y);

            telemetry.addData("Target tag", auto.cameraHardware.getTargetTag());
            telemetry.addData("Num tag we see", auto.cameraHardware.getNumSeenTags());
            telemetry.addData("Did we fucking relocalize???", didWeFuckingRelocalize);

            telemetry.addData("armHeight", auto.armHeight);
            //telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);

            packet = new TelemetryPacket();
            for (Point point: trajectoryBeingFollowed.getFullCurve())
            {
                //telemetry.addData("POINT", point);
                packet.fieldOverlay().setFill("black").fillCircle(point.x, point.y, 1);

            }
            for (Point point : pathTraveled)
            {
                packet.fieldOverlay().setFill("Orange").fillCircle(point.x, point.y, 0.8);
            }
            packet.fieldOverlay().setStroke("Blue").strokeRect(drive.getLocalizer().getPredictedPoseEstimate().getX() - 7.5, drive.getLocalizer().getPredictedPoseEstimate().getY() - 8, 14.5, 16);
            packet.fieldOverlay().setStroke("Red").strokeRect(drive.getLocalizer().getPoseEstimate().getX() - 7.5, drive.getLocalizer().getPoseEstimate().getY() - 8, 14.5, 16);
            pathTraveled.add(drive.getLocalizer().getPoseEstimate().toPoint());


            drive.update();
            dashboard.sendTelemetryPacket(packet);

        }
        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
        auto.storePoseEndAuto(poseEstimate);
    }

    public void autoSequence(){

        if (auto.goToPark(currentState == AutoState.IDLE,2)){
            currentState = AutoState.IDLE;
        }

        switch (currentState) {
            case DELAY_BACK:
                if(auto.delayState(!isArmDown? 100:0)){
                    currentState = AutoState.PRELOAD_DRIVE_BACK;
                    teamPropLocation = 1;
                    if (teamPropLocation == 1){
//                        drive.setTargetPose(new Pose(auto.autoTrajectories.PreloadDrive1.end()));
                        //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1);
                        drive.followTrajectory(purpleYellowTrajectory);
                        trajectoryBeingFollowed = purpleYellowTrajectory;

                    } else if (teamPropLocation == 2){
                        //drive.setTargetPose(new Pose(auto.autoTrajectories.PreloadDrive2.end()));
                        //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2);
                    } else if (teamPropLocation == 3){
                        //drive.setTargetPose(new Pose(auto.autoTrajectories.PreloadDrive3.end()));
                        //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3);
                    }
                }
                break;
            case PRELOAD_DRIVE_BACK:
                if(auto.preloadDriveStateBack()){
                    currentState = AutoState.PLACE_AND_INTAKE_BACK;
                }
                break;
            case PLACE_AND_INTAKE_BACK:
                if (auto.placeAndIntakeBackSTage(0.8)){
                    currentState = AutoState.DROP;
                }
                break;

            case DELAY:

                break;

            case PRELOAD_DRIVE_CASE_3:
                if (auto.preloadDriveState3()){
                    currentState = AutoState.AFTER_PRELOAD_DRIVE_3;
                    //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontSecond);
                    //drive.setTargetPose(new Pose(auto.autoTrajectories.PreloadDrive3FrontSecond.end()));
                }
                break;

            case AFTER_PRELOAD_DRIVE_3:
                if (auto.afterPreloadDriveState3()){
                    currentState = AutoState.PLACE_AND_INTAKE;
                    auto.frontThirdCase = true;
                }
                break;

            case PRELOAD_DRIVE:
                if(auto.preloadDriveState(false, true,950, 0.18, teamPropLocation == 2 || teamPropLocation == 3)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;

            case PLACE_AND_INTAKE:
                if (auto.placeAndIntakeFrontMIDTRUSS(350,0.18, true)){
                    if (auto.goBackForYellowPixel){
                        currentState = AutoState.GO_BACK_FOR_YELLOW;
                    } else {
                        if (auto.GlobalTimer.seconds() > delayForYellow){
                            Trajectory startDrive = null;
                            if (teamPropLocation == 2){
                                //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                            } else if (teamPropLocation == 1){
                                trajectoryBeingFollowed = firstIntakeTrajectory;
                                drive.followTrajectory(firstIntakeTrajectory);
                                //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                            } else if (teamPropLocation == 3){
                                //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
                            }
                            //auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                            currentState = AutoState.TRANSFER_PIXEL;
                        }
                    }
                }
                break;

            case GO_BACK_FOR_YELLOW:
                if (auto.reExtendSLidesForYellow(1200,0.35)){
                    if (auto.GlobalTimer.seconds() > delayForYellow){
                        Trajectory startDrive = null;
                        if (teamPropLocation == 2){
                            //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                        } else if (teamPropLocation == 1){
                            //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                        } else if (teamPropLocation == 3){
                            //startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
                        }
                        //auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                        //drive.setTargetPose(startDrive.end());
                        currentState = AutoState.TRANSFER_PIXEL;
                    }
                }
                break;

            case TRANSFER_PIXEL:
                boolean gripEarly = true;
                if (numCycles == 0 && frontOrBackAuto){
                    gripEarly = false;
                    if (xPosition>-50 && xPosition<10){
                        auto.intakeSubsystem.intakeSpin(-1);
                    }
                }
                if (numCycles < 3){
                    //auto.goBackToStack(3,6,-29.3);
                }
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                if (auto.transferPixel(gripEarly)){
                    currentState = AutoState.OUTTAKE_PIXEL;
                }
                break;

            case OUTTAKE_PIXEL:
                //outtaking lengths for each cycle
                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 330; // pre-extend intake for most cycles
                com.acmerobotics.roadrunner.trajectory.Trajectory intakeTrajectory = null;
                boolean openGrippers = true;
                boolean extendStraightAway = false;
                if (xPosition > -7 && numCycles != 0){ // custom extend on the first cycle
                    auto.autoTrajectories.extendSlidesAroundTruss = true;
                }
                if (numCycles == 0){
                    intakeSlideTarget = 10;
                    if (xPosition > 4){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.GRIP);
                    }
                    if (xPosition > 18){ //teamPropLocation != 1? xPosition > 21: teamPropLocation == 2? xPosition > 19:
                        // stop yellow detection
                        auto.cameraHardware.pausePreloadProcessor();
                        railTarget = auto.cameraHardware.getRailTarget(auto.correctedHeading, ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition);
                        railLogic.setRailTargetFromAprilTag(railTarget);
                    }

                    pitchTarget = 31; // yellow pixel should be pitched higher
                    liftTarget = 13.2;
                    openGrippers = false;
                } else if (numCycles == 1)
                {
                    pitchTarget = 19;
                    liftTarget = 27;
                } else if (numCycles == 2){
                    pitchTarget = 23;
                    liftTarget = 30.5;
                } else if (numCycles == 3){
                    pitchTarget = 25;
                    liftTarget = 31;
                    intakeSlideTarget = 70;
                } else if (numCycles == 4){
                    pitchTarget = 27;
                    liftTarget = 31;
                    intakeSlideTarget = 70;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,
                        intakeSlideTarget,railLogic,pivotLogic,extendStraightAway,
                        true, false, openGrippers, true);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished){ // if team prop location is 1 we don't want more pixels //|| (teamPropLocation == 1 && numCycles == 3)
                    //intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
                    currentState = AutoState.PARK;
                    //auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                }
                else if (outtakePixelFinished){

                    if (numCycles == 1){
                        intakeTrajectory = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,5.2);
                    }
                    if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(new Pose2d(xPosition+1.8,yPosition,headingPosition),numCycles == 1? 10:22,3,2.3 + S == -1?0:0,-27.5, -25.2, S == 1? 180:180);
                    } else if (numCycles == 3 || numCycles == 4){ // turning into the stacks
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(new Pose2d(xPosition+2.7,yPosition,headingPosition),19,-3,endAngleForStacks,3,3.8 + S == -1?-1.4:0,-18);
                    }
                    //TODO mental note - if you move the x distance upwards the angle needs to be less and the offset needs to be more for the spline to work properly
else if (numCycles == 4){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStage(poseEstimate,22,-2,endAngleForStacks,3,0,-23);
                    }

                    didWeFuckingRelocalize = auto.resetPosWithAprilTags(3,S == -1? false:true);
                    trajectoryBeingFollowed = firstIntakeTrajectory;
                    drive.followTrajectory(firstIntakeTrajectory);

                    currentState = AutoState.DROP;
                }
                break;
            case DROP:
                Trajectory intakeTrajectoryAfterDrop;
                double delayTime = 190;
                int armHeight = 0;
                if (numCycles == 1){
                    if (auto.delay(500)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
                        if(auto.delay(710)){
                            auto.outtakeSubsystem.liftToInternalPID(0,0.4);
                        } // so we don't rely on a drive back
                    }
                    if(auto.delay(270) && !frontOrBackAuto){
                        auto.outtakeSubsystem.liftToInternalPID(0,0.5);
                    }
                    delayTime = frontOrBackAuto? (teamPropLocation == 3?830:690):300;
                    if (frontOrBackAuto){
                        armHeight = 4;
                    } else {
                        armHeight = 5;
                    }
                } else if (numCycles == 2){
                    if (frontOrBackAuto){
                        armHeight = 1;
                    } else {
                        armHeight = 3;
                    }
                } else if (numCycles == 3){
                    armHeight = 5;
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, false, delayTime)){
                    if (numCycles == 1){ // for very first cycle
                        if (frontOrBackAuto){
                            if (teamPropLocation == 1){
                                trajectoryBeingFollowed = firstIntakeTrajectory;
                                drive.followTrajectory(firstIntakeTrajectory);
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterYellowStage1.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage1);
                            } else if (teamPropLocation == 2){
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterYellowStage2.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage2);
                            } else if (teamPropLocation == 3){
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterYellowStage3.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage3);
                            }
                        } else { // for the back side autos we just run this straight away
                            if (teamPropLocation == 1){
                                trajectoryBeingFollowed = firstIntakeTrajectory;
                                drive.followTrajectory(firstIntakeTrajectory);
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterBackStage1.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage1);
                            } else if (teamPropLocation == 2){
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterBackStage2.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage2);
                            } else if (teamPropLocation == 3){
                                //drive.setTargetPose(auto.autoTrajectories.driveIntoStacksAfterBackStage3.end());
                                //auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage3);
                            }
                        }
                    }
                    if (auto.GlobalTimer.seconds() > 25.1 || (teamPropLocation == 1 && frontOrBackAuto && numCycles ==3)){
                        auto.parkIfStuck = true; // should force into park before doing another cycle
                    } else {
                        currentState = AutoState.GRAB_OFF_STACK;
                    }
                }
                break;
            case GRAB_OFF_STACK:
                if ((xPosition < -18) && ((endAngleForStacks - Math.toDegrees(headingPosition)) < 2.8)){ // test to see if this works
                    auto.autoTrajectories.extendSlidesAroundStage = true;
                }
                double delayBeforeRetracting = 200;
                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
                boolean extendSlides = false;
                double xPosSlideThresh = -10;
                boolean retractSlides = false;
                double xSplineValue = 6;
                double yOffset = 4;
                double endTangent = -6;
                double slideSpeed = 1;
                double xEnd = 29.2;

                if (numCycles == 1){
                    delayBeforeRetracting = 600;
                    retractSlides = true;
                    slideSpeed = 0.8;
                    if (xPosition < 8){//xPosition < -17
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
yOffset = 4;
                    xSplineValue = 6;

 if (S == 1){
                        extendSlides = true;
                    }

                }
                if (numCycles == 2){
                    slideSpeed = 0.83;
                    if (S == 1? xPosition < 12 : xPosition < 12){ //  xPosition < -14
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
if (S == 1){
                        extendSlides = true;
                    }

                }
                if (numCycles == 3){
                    //slideSpeed = 1;
                    xEnd = 30.5;
                    intakeSlidePosition = 838;
                    xSplineValue = 3;
                    yOffset = 4.3;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.3;
                } else if (numCycles == 4){
                    xEnd = 32;
                    intakeSlidePosition = 850;
                    xSplineValue = 3;
                    yOffset = 4.5;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.3;
                }
                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition, delayBeforeRetracting, xPosSlideThresh, retractSlides, slideSpeed)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;

                    Trajectory outtakeTrajectory = null;
   if (numCycles == 1){
                    }
                    if (numCycles == 2){
                        //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 169, 4.8,14);
                    } else if (numCycles == 3){
                        //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,16, 168, 5.4,12);
                    } else if (numCycles == 4){
                        //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,16, 168, 5.4,12);
                    }



   if (numCycles >= 3) { // for the longer delay we follow the trajectory after the wait - just so its more consistent hopefully
                          //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromAngleTurnEndTrajectory(poseEstimate, 20, 31, endTangent, -yOffset, 3,-3,10);
                          //outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,20, 173, 4.3);
                       //   auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                      }
                    //  else{// (numCycles < 4) {
                    // this is the old spline path
                    else {
                          //outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 22 : 19, 176.8, endTangent, yOffset, xSplineValue);
                         }

                    //outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 21 : 22, 176.8, endTangent, yOffset, xSplineValue, xEnd);
                    //      outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 175, 4);
                    //     outtakeTrajectory = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,18, 175, 4);
                    //   }

                    trajectoryBeingFollowed = depositTrajectory;
                    drive.followTrajectory(depositTrajectory);

                }
                break;
            case AFTER_GRAB_OFF_STACK:
                if (auto.afterGrabOffStack(2,3, 365,300)){
                    currentState = AutoState.TRANSFER_PIXEL;
                }
                break;
            case PARK:
                if (auto.park()){
                    currentState = AutoState.IDLE;
                }
                break;
            case IDLE:
                auto.idle();
                break;
        }

    }
}
