package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.SetAuto;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoTrajectories.*;

@Autonomous(name = "Back Blue Stage Auto", group = "Autonomous")
public class Back_BLUE_Stage extends LinearOpMode {

    int numCycleForDifferentLane = 0;
    double delayForYellow = 0; // this is in seconds
    double endAngleForStacks = S == 1? -165: -164;
    boolean didWeFuckingRelocalize = false;
    Trajectory outtakeTrajectory;
    double initialDelay = 0;

    //Accessories
    AutoSequences auto = new AutoSequences(telemetry,3);
    LoopTime loopTime = new LoopTime();
    // this works because the parameters being passed in don't change throughout the opmode
    // same cycle numbers
    AutoRail railLogic = new AutoRail(numCycleForDifferentLane,0, auto,true,3);
    AutoPivot pivotLogic = new AutoPivot(numCycleForDifferentLane,0, auto, telemetry,3);
    private double railTarget;


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

    @Override
    public void runOpMode() throws InterruptedException {

        frontOrBackAuto = false;
        SetAuto.setBlueAuto();
        auto.setGamepad1(gamepad1);
        if (!frontOrBackAuto){
            isArmDown = true;
        }

        DriveConstants.MAX_VEL = 60;
        DriveConstants.MAX_ACCEL = 52;

        if (S == -1){
            auto.autoTrajectories.MiddleLaneYIntake -= 3.3;
            auto.autoTrajectories.StageYIntake -= 3.9;
        }

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) { // turns on bulk reads cannot double read or it will call multiple bulkreads in the one thing
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.clearBulkCache();
        } //

        auto.initAutoHardware(hardwareMap,this);

        // trajectories that aren't changing should all be here
        while (!isStarted()) { // initialization loop
            telemetry.addData("middleStageYIntake", auto.autoTrajectories.MiddleLaneYIntake + LaneOffset);
            telemetry.addData("StageYIntake", auto.autoTrajectories.StageYIntake);
            auto.intializationLoop(!isArmDown);
            auto.setPlaceForBack();
            if (teamPropLocation == 1){
                telemetry.addLine("Front");
            } else if (teamPropLocation == 2){
                telemetry.addLine("Middle");
            } else if (teamPropLocation == 3){
                telemetry.addLine("Back");
            }
            telemetry.addData("S", S);

            if(gamepad1.a)
            {
                initialDelay = 1000;
            }
            if(gamepad1.x)
            {
                initialDelay = 2000;
            }
            if(gamepad1.y)
            {
                initialDelay = 3000;
            }
            if(gamepad1.b)
            {
                initialDelay = 5000;
            }
            telemetry.addData("Initial Delay", initialDelay);
            telemetry.update();
        }

        Log log = new Log("MotorDraw", true);
        waitForStart();
        if (isStopRequested()) return;
        // runs instantly once
        auto.afterWaitForStart(!frontOrBackAuto, frontOrBackAuto? auto.autoTrajectories.startPoseFront: auto.autoTrajectories.startPoseBack);
        if (frontOrBackAuto){
            currentState = AutoState.DELAY;
        } else {
            currentState = AutoState.DELAY_BACK;
        }

        // can set drive constraints here
        while (opModeIsActive() && !isStopRequested()) {
            // Reading at the start of the loop
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
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
            telemetry.addData("numCycles", numCycles);
            //telemetry.addData("Preload", auto.cameraHardware.getPreloadYellowPose());
            telemetry.addData("LoopTime", loopTime.getDt() / 1_000_000);
            //telemetry.addData("Hz", loopTime.getHz());
            telemetry.addData("Auto State", currentState);
            telemetry.addData("middleStageYIntake", auto.autoTrajectories.MiddleLaneYIntake);
            telemetry.addData("StageYIntake", auto.autoTrajectories.StageYIntake);
            //telemetry.addData("extend slides", auto.autoTrajectories.extendSlidesAroundStage);
            //telemetry.addData("extend slides around truss", auto.autoTrajectories.extendOuttakeSlidesAroundTruss);
//            log.addData(
//                    auto.outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS),
//                  //  auto.outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS),
//                  //  auto.intakeSubsystem.IntakeMotor.getCurrent(CurrentUnit.AMPS),
//                    auto.intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
//            log.update();

            //telemetry.addData("X OFfset", auto.cameraHardware.ROBOT_X);
            //telemetry.addData("Y Offset", auto.cameraHardware.ROBOT_Y);

//            telemetry.addData("Target tag", auto.cameraHardware.getTargetTag());
//            telemetry.addData("Num tag we see", auto.cameraHardware.getNumSeenTags());
            //telemetry.addData("Did we fucking relocalize???", didWeFuckingRelocalize);

            //telemetry.addData("armHeight", auto.armHeight);
            //telemetry.addData("intakeSlidePosition", auto.intakeSubsystem.intakeSlidePosition);


            // TODO this will tank our looptimes
//            telemetry.addData("IntakeSlideMotor Current", auto.intakeSubsystem.IntakeSlideMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LiftMotor Current", auto.outtakeSubsystem.LiftMotor.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("PitchMotor Current", auto.outtakeSubsystem.PitchMotor.getCurrent(CurrentUnit.AMPS));


            telemetry.update();

        }
        //auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);
        auto.storePoseEndAuto(poseEstimate);
        log.close();
    }

    public void autoSequence(){

        if (auto.goToPark(currentState == AutoState.IDLE || currentState == AutoState.DROP || currentState == AutoState.OUTTAKE_PIXEL ,2)){
            currentState = AutoState.IDLE;
        }

        switch (currentState) {
            case DELAY_BACK:
                if(auto.delayState(!isArmDown? 100:0)){
                    currentState = AutoState.PRELOAD_DRIVE_BACK;
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1);
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2);

                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3);
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
                if(auto.delayState(initialDelay)){
                    if (teamPropLocation == 1){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive1FrontStage);
                    } else if (teamPropLocation == 2){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive2FrontFirst);
                    } else if (teamPropLocation == 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.PreloadDrive3FrontFirst);
                    }
                    if (teamPropLocation == 1){
                        currentState = AutoState.PRELOAD_DRIVE;
                    } else {
                        currentState = AutoState.PRELOAD_DRIVE_CASE_3;
                    }

                }
                break;

            case PRELOAD_DRIVE_CASE_3:
                if (auto.preloadDriveState3()){
                    currentState = AutoState.AFTER_PRELOAD_DRIVE_3;
                    Trajectory startDrive = null;
                    if (teamPropLocation == 2){
                        startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2StraightTo;
                    } else if (teamPropLocation == 3){
                        startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3StraightTo;

                    }
                    auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);

                }
                break;

            case AFTER_PRELOAD_DRIVE_3:
                if (auto.afterPreloadDriveState3()){
                    currentState = AutoState.TRANSFER_PIXEL;
                    auto.cameraHardware.pausePreloadProcessor();
                    auto.frontThirdCase = true;
                }
                break;

            case PRELOAD_DRIVE:
                if(auto.preloadDriveState(false, false,0, 0.18, teamPropLocation == 2 || teamPropLocation == 3)){
                    currentState = AutoState.PLACE_AND_INTAKE;
                }
                break;

            case PLACE_AND_INTAKE:

                if (auto.placeAndIntakeFrontMIDTRUSS(350,0.23, true)){
                    if (auto.goBackForYellowPixel){
                        currentState = AutoState.GO_BACK_FOR_YELLOW;
                    } else {
                        if (auto.GlobalTimer.seconds() > delayForYellow){
                            Trajectory startDrive = null;
                            if (teamPropLocation == 2){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                            } else if (teamPropLocation == 1){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                            } else if (teamPropLocation == 3){
                                startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3StraightTo;
                            }
                            auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                            currentState = AutoState.TRANSFER_PIXEL;
                            auto.cameraHardware.pausePreloadProcessor();
                        }
                    }
                }
                break;

            case GO_BACK_FOR_YELLOW:
                if (auto.reExtendSLidesForYellow(1200,0.23)){
                    if (auto.GlobalTimer.seconds() > delayForYellow){
                        Trajectory startDrive = null;
                        if (teamPropLocation == 2){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple2;
                        } else if (teamPropLocation == 1){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple1;
                        } else if (teamPropLocation == 3){
                            startDrive = auto.autoTrajectories.firstDriveThroughStageAfterPurple3;
                        }
                        auto.autoTrajectories.drive.followTrajectoryAsync(startDrive);
                        currentState = AutoState.TRANSFER_PIXEL;
                        auto.cameraHardware.pausePreloadProcessor();
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
                    auto.goBackToStack(3,8,-29.3, 180);
                } else if (numCycles >= 3){
                    auto.goBackToStack(3,20,-33, endAngleForStacks+3);
                    if (xPosition < 17){
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
                }
                if (auto.goBackToStack){
                    currentState = AutoState.GRAB_OFF_STACK;
                }
                if (auto.transferPixel(gripEarly)){
                    currentState = AutoState.OUTTAKE_PIXEL;
                }
                break;

            case OUTTAKE_PIXEL:
                telemetry.addData("rail target", railTarget);
                //outtaking lengths for each cycle
                double positiveYOffset = 0;
                if (frontOrBackAuto){
                    positiveYOffset = -1;
                } else {
                    positiveYOffset = 0;
                }
                double liftTarget = 0; // could cause issues if these stay zero
                int pitchTarget = 0;
                int intakeSlideTarget = 330; // pre-extend intake for most cycles
                Trajectory intakeTrajectory = null;
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
                    if (xPosition > 12){ //teamPropLocation != 1? xPosition > 21: teamPropLocation == 2? xPosition > 19:
                        // stop yellow detection
                        railTarget = auto.cameraHardware.getRailTarget(auto.correctedHeading, ticksToInchesSlidesMotor(auto.outtakeSubsystem.liftPosition), auto.outtakeSubsystem.pitchEncoderPosition);
                        railLogic.setRailTargetFromAprilTag(railTarget);
                    }

                    pitchTarget = 27; // yellow pixel should be pitched higher
                    liftTarget = 13;
                    openGrippers = false;
                } else if (numCycles == 1)
                {
                    pitchTarget = 23;
                    liftTarget = 25.45;
                } else if (numCycles == 2){
                    pitchTarget = 26;
                    liftTarget = 28;
                } else if (numCycles == 3){
                    pitchTarget = 31;
                    liftTarget = 28;
                    intakeSlideTarget = 70;
                } else if (numCycles == 4){
                    pitchTarget = 30;
                    liftTarget = 28;
                    intakeSlideTarget = 70;
                    auto.goToParkAfterOuttaking = true;
                }
                boolean outtakePixelFinished = auto.outtakePixel(auto.correctedHeading,liftTarget,pitchTarget,
                        intakeSlideTarget,railLogic,pivotLogic,extendStraightAway,
                        false, false, openGrippers, true);
                if (auto.goToParkAfterOuttaking && outtakePixelFinished){ // if team prop location is 1 we don't want more pixels //|| (teamPropLocation == 1 && numCycles == 3)
                    intakeTrajectory = auto.autoTrajectories.parkTrajectory(poseEstimate,2);
                    currentState = AutoState.PARK;
                    auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                }

                else if (outtakePixelFinished){//
                    if (numCycles == 1){
                        // intakeTrajectory = auto.autoTrajectories.driveBackToDropYellow(poseEstimate,10,5.6);
                    }
                    if (numCycles == 2){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackStraightTrajectory(new Pose2d(xPosition+0.5,yPosition,headingPosition),22,3,3.05 + positiveYOffset + (S == -1?1.3:0),-27.7, -26.3, S == 1? 180:180);
                    } else if (numCycles == 3){ // turning into the stacks
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStageV2(new Pose2d(xPosition + 1,yPosition,headingPosition),45,-2.3,endAngleForStacks,-172.5, (S == -1?2.27:0.58) + positiveYOffset,5);
                    }
                    //TODO mental note - if you move the x distance upwards the angle needs to be less and the offset needs to be more for the spline to work properly
                    else if (numCycles == 4 && !frontOrBackAuto){
                        intakeTrajectory = auto.autoTrajectories.driveIntoStackAngledAfterAngledOuttakeTrajectoryStageV2(poseEstimate,45,-2.3,endAngleForStacks,-172.5,0.58,5);
                    }
                    // if (numCycles != 0){
                    if (numCycles == 0 && teamPropLocation == 1 && frontOrBackAuto){
                        // dont relocalize here...
                    } else {
                        didWeFuckingRelocalize = auto.resetPosWithAprilTags(3,S == -1? true:true);
                    }
                    //}

                    if (intakeTrajectory != null){
                        auto.autoTrajectories.drive.followTrajectoryAsync(intakeTrajectory);
                    }
                    currentState = AutoState.DROP;
                }
                break;

            case DROP:
                Trajectory intakeTrajectoryAfterDrop;
                double delayTime = 190;
                int armHeight = 1;
                if (numCycles == 1){
                    if (auto.delay(200)){
                        auto.outtakeSubsystem.gripperServoState(OuttakeSubsystem.GripperServoState.OPEN);

                    }
                    if(auto.delay(200) && !frontOrBackAuto){
                        auto.outtakeSubsystem.pitchToInternalPID(25,0.5);
                        auto.outtakeSubsystem.liftToInternalPID(0,0.8);
                        telemetry.addLine("RETRACT THE LIFT");
                    }

                    delayTime = frontOrBackAuto? 350:300;
                    if (frontOrBackAuto){
                        //if (teamPropLocation != 1){
                        armHeight = 5;
                        // } else {
                        //     armHeight = 4;
                        // }
                    } else {
                        armHeight = 5;
                    }
                } else if (numCycles == 2){
                    if (frontOrBackAuto){
                        armHeight = 3;
                    } else {
                        armHeight = 3;
                    }
                } else if (numCycles == 3){
                    if (frontOrBackAuto){
                        armHeight = 4;
                    } else {
                        armHeight = 5;
                    }
                } else if (numCycles == 4) {
                    armHeight = 3;
                }

                if (auto.drop(armHeight, false, delayTime)){
                    if (numCycles == 1){ // for very first cycle
                        if (frontOrBackAuto){
                            if (teamPropLocation == 1){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage1);
                            } else if (teamPropLocation == 2){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage2);
                            } else if (teamPropLocation == 3){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterYellowStage3);
                            }
                        } else { // for the back side autos we just run this straight away
                            if (teamPropLocation == 1){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage1);
                            } else if (teamPropLocation == 2){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage2);
                            } else if (teamPropLocation == 3){
                                auto.autoTrajectories.drive.followTrajectoryAsync(auto.autoTrajectories.driveIntoStacksAfterBackStage3);
                            }
                        }
                    }
                    if (auto.GlobalTimer.seconds() > 25.1){
                        auto.parkIfStuck = true; // should force into park before doing another cycle
                    } else {
                        currentState = AutoState.GRAB_OFF_STACK;
                    }
                }
                break;
            case GRAB_OFF_STACK:
                if ((xPosition < -25) ){ // && ((endAngleForStacks - Math.toDegrees(headingPosition)) < 2.5)
                    auto.autoTrajectories.extendSlidesAroundStage = true;
                }
                double delayBeforeRetracting = 200;
                int intakeSlidePosition = INTAKE_SLIDE_AUTO_LONG_PRESET;
                boolean extendSlides = false;
                double xPosSlideThresh = -10;
                boolean retractSlides = false;
                double xSplineValue = 5;
                double yOffset = 5.5;
                double endTangent = -6.8;
                double slideSpeed = 1;
                double xEnd = 29.2;

                if (numCycles == 1){
                    xEnd = 29;
                    delayBeforeRetracting = 650;
                    retractSlides = true;
                    slideSpeed = 0.8;
                    if (xPosition < 8){//xPosition < -17
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }
                    /*yOffset = 4;
                    xSplineValue = 6;*/
                   /* if (S == 1){
                        extendSlides = true;
                    }*/
                }
                if (numCycles == 2){
                    slideSpeed = 0.8;
                    xPosSlideThresh = -12;
                    xEnd = 29.8;
                    yOffset = 4.3;
                    endTangent = -7;
                    if (S == 1? xPosition < 12 : xPosition < 12){ //  xPosition < -14
                        auto.autoTrajectories.extendSlidesAroundStage = true;
                    }/*if (S == 1){
                        extendSlides = true;
                    }*/
                }
                if (numCycles == 3){
                    //slideSpeed = 1;
                    xEnd = 31.7;
                    if (frontOrBackAuto){
                        intakeSlidePosition = 970;
                    } else {
                        intakeSlidePosition = 960;
                    }
                    xSplineValue = 3;
                    yOffset = 5;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.3;
                } else if (numCycles == 4){
                    xEnd = 32.3;
                    intakeSlidePosition = 990;
                    xSplineValue = 3;
                    yOffset = 5.1;
                    extendSlides = false;
                    retractSlides = true;
                    endTangent = -7.6;
                }
                if (auto.grabOffStack(numCycleForDifferentLane, true, extendSlides,3, intakeSlidePosition, delayBeforeRetracting, xPosSlideThresh, retractSlides, slideSpeed)){
                    currentState = AutoState.AFTER_GRAB_OFF_STACK;


                    outtakeTrajectory = auto.autoTrajectories.simplifiedOuttakeDrive(poseEstimate, numCycles >= 3 ? 20 : 21, 176.8, endTangent, yOffset, xSplineValue, xEnd);
                    //      outtakeTrajectory = auto.autoTrajectories.outtakeDriveFromStraightTUrnEndStageV2Trajectory(poseEstimate,18, 175, 4);
                    //     outtakeTrajectory = auto.autoTrajectories.outtakeDriveMiddlePathTrajectory(poseEstimate,18, 175, 4);
                    //   }
                    if (numCycles < 3){
                        auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
                    }
                }
                break;
            case AFTER_GRAB_OFF_STACK:
                double intakeTime = 0;
                if (numCycles == 1) intakeTime = 270;
                if (numCycles == 2) intakeTime = 300;
                if (auto.afterGrabOffStack(2,3, 500,intakeTime)){
                    if (!auto.intakeSubsystem.pixelsInIntake() && numCycles >= 3){
                        currentState = AutoState.GO_BACK_FOR_WHITES;
                    } else {
                        currentState = AutoState.TRANSFER_PIXEL;
                        if (numCycles >= 3) auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);

                    }
                }
                break;
            case GO_BACK_FOR_WHITES:
                if(auto.reExtendSlidesForStage()){
                    currentState = AutoState.TRANSFER_PIXEL;
                    auto.autoTrajectories.drive.followTrajectoryAsync(outtakeTrajectory);
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