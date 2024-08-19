package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.system.accessory.Tags;
import org.firstinspires.ftc.teamcode.system.accessory.math.MathResult;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

@Config // Allows dashboard tune
public class Globals
{

    public static double PITCH_OFFSET = 2.8;
    //public static double BLUE_AUTO_OFFSET = 1.9;
    public static boolean isArmDown;
    public static boolean frontOrBackAuto;

    // Auto Constants
    public static double
            autoTimer,
            stateTimer,
            numCycles = 0;

    public static double
            RAIL_CENTER_POS = 0.487,

            RAIL_LEFT_LESS_POS = RAIL_CENTER_POS + 0.25,
            RAIL_RIGHT_LESS_POS = RAIL_CENTER_POS - 0.1,


            RAIL_CENTER_YELLOW_POS = RAIL_CENTER_POS - 0.067,
            RAIL_CENTER_YELLOW_TRUSS_POS = RAIL_CENTER_POS + 0.36,
            RAIL_CENTER_YELLOW_STAGE_POS = RAIL_CENTER_POS - 0.23,

            RAIL_RIGHT_POS = 0,
            RAIL_RIGHT_YELLOW_POS = RAIL_CENTER_POS - 0.297,
            RAIL_RIGHT_YELLOW_TRUSS_POS = RAIL_CENTER_POS + 0.297,
            RAIL_RIGHT_YELLOW_STAGE_POS = RAIL_CENTER_POS - 0.297,

            RAIL_LEFT_POS = 1,
            RAIL_LEFT_YELLOW_POS = RAIL_CENTER_POS + 0.313,
            RAIL_LEFT_YELLOW_TRUSS_POS = RAIL_CENTER_POS + 0.297,
            RAIL_LEFT_YELLOW_STAGE_POS = RAIL_CENTER_POS - 0.297,
            OUTTAKE_DISTANCE_AUTO_THRESHOLD = 8.8;


    // Auto States
    public static boolean
            BLUE_AUTO = false,

            RED_AUTO = false;

    public enum Place
    {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }

    public static Place place = Place.NONE;

    // Team Prop Location
    public static int teamPropLocation = 1;
    public static LynxModule chub, expHub;
    public static double EPSILON_DELTA = 0.005;

    // Pitch
    public static double
            //PITCH_TICKS_PER_REVOLUTION = 3895.9,
            //PITCH_TICKS_PER_DEGREES = PITCH_TICKS_PER_REVOLUTION / 360,

            TICKS_PER_BAREMOTOR = 28,

            PITCH_TICKS_PER_REVOLUTION = 3892,
            PITCH_TICKS_PER_DEGREES = ((PITCH_TICKS_PER_REVOLUTION / 0.8) / 360),


            GAMEPAD_TRIGGER_THRESHOLD = 0.2,

            RAIL_SERVO_POSITION,
            FINE_ADJUST_HEIGHT_INTERVAL = 3,
            MIN_OUTTAKE_EXTENSION_INCHES = 15.5,
            A; // angle offset for autonmous

    public static int
            PITCH_OFFSET_VALUE = 319,
            PITCH_DEFAULT_DEGREE_TICKS = 37,
            PITCH_CLIMB_TICKS = 67,
            UPRIGHT_PITCH_TICKS = 56,
            PITCH_LOW_DEGREE_TICKS = 22,
            PITCH_PURPLE_PIXEL_POSITION = 23,
            PITCH_MID_DEGREE_TICKS = 45,

            INTAKE_SLIDE_EXTENDO_TELEOP_FAR = 1000,
            INTAKE_SLIDE_EXTENDO_TELEOP_CLOSE = 500,

            INTAKE_SLIDE_AUTO_LONG_PRESET = 1000,

            LIFT_INCHES_FOR_MAX_EXTENSION = 25, // 28 is the actual max but we need to give some adjustment room

            // Nats shit bro kinda L
            //LIFT_HIGH_POSITION_TICKS = 600,
            //LIFT_MEDIUM_POSITION_TICKS = 400,
            //LIFT_LOW_POSITION_TICKS = 70,

            //LIFT_MEDIUM_POSITION_PITCHING_TICKS = 580,
            //LIFT_LOW_POSITION_PITCHING_TICKS = 690,
            LIFT_HITS_WHILE_PITCHING_THRESHOLD = 3,

            // Worlds constants
            HIGH_BACKDROP_PRESET_INCHES = 36,
            MID_BACKDROP_PRESET_INCHES = 21,
            LOW_BACKDROP_PRESET_INCHES = 14,
            S, // the side multiplier to change sides for autonomous
            LEFT_OR_RIGHT;


    // this custom position library credit Michael from team 14343 (@overkil on Discord)
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    public static double degreestoTicksPitchMotor(double degrees){
        return (degrees * PITCH_TICKS_PER_DEGREES)/0.389921;
    }

    public static double ticksToDegreePitchMotor (double ticks){
        return (ticks/PITCH_TICKS_PER_DEGREES)*0.389921;
    }

    public static double ticksToInchesSlidesMotor(double ticks){
        return ((1.33858*Math.PI)/(TICKS_PER_BAREMOTOR*5)) * ticks; //34mm is the diameter of the reel
    }

    public static double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BAREMOTOR*5)/(1.33858*Math.PI)) * inches; //ticks per inches
        // ratio is 60/12 = 5
    }

    // Inverse Kinematic Constants
    public static double
            APRILTAG_POSE_X, // Apriltag coordinates from camera
            APRILTAG_POSE_Y,
            APRILTAG_POSE_Z,

            PIXEL_DISTANCE_X = 0, // Distance from apriltag to pixel scoring
            PIXEL_DISTANCE_Z = 12,

            PIXEL_PITCH_DEGREES = 30, // Scoring pixel pitch (Backdrop pitch)
            PIXEL_PITCH_RADIANS = Math.toRadians(PIXEL_PITCH_DEGREES),
            PIXEL_DISTANCE_Y = PIXEL_DISTANCE_Z *Math.tan(PIXEL_PITCH_RADIANS),

            PIXEL_POSE_X, // Pixel scoring coordinates
            PIXEL_POSE_Y,
            PIXEL_POSE_Z;

    public static double angleWrap(double radians) {
        if (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        else if (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }

    public static double normalizeRadians(double radians)
    {
        radians %= 2.0 * Math.PI;
        if (radians < -Math.PI) radians += 2.0 * Math.PI;
        if (radians > Math.PI) radians -= 2.0 * Math.PI;
        return radians;
    }

    public static double angleWrapDegrees(double degrees) {
        if (degrees > 180) {
            degrees -= 360;
        }
        else if (degrees < -180) {
            degrees += 360;
        }

        // keep in mind that the result is in degrees
        return degrees;
    }

    /**
     * @param current          current power returned from the pid
     * @param prev             previous power set to the motor
     * @param cachingTolerance threshold for the abs difference between the powers
     * @param motor            the motor the power should bet set if one of the conditions is true
     * @return the value to be cached as the previous power
     */
    public static double motorCaching(double current, double prev, double cachingTolerance, DcMotor motor)
    {
        // This can also be done as wrapper to the hardware object
        if (
                (Math.abs(current - prev) > cachingTolerance) ||
                        (current == 0.0 && prev != 0.0) ||
                        (current >= 1.0 && !(prev >= 1.0)) ||
                        (current <= -1.0 && !(prev <= -1.0))
        )
        {

            prev = current;
            motor.setPower(current);
        }
        return prev;
    }

    public static int TargetCaching(int target, int prevTarget, double cachingTolerance, DcMotor motor, boolean firstCycle)
    {
        if (
                (Math.abs(target - prevTarget) > cachingTolerance) || firstCycle
        )
        {
            motor.setTargetPosition(target);
            prevTarget = target;
        }

        return prevTarget;
    }

    public static double servoCaching(double current, double prev, double cachingTolerance, ServoImplEx servo)
    {
        if (
                (Math.abs(current - prev) > cachingTolerance) ||
                        (current == 0.0 && prev != 0.0) ||
                        (current >= 1.0 && !(prev >= 1.0))
        )
        {

            prev = current;
            servo.setPosition(current);
        }

        return prev;
    }
    public static DcMotor.RunMode runModeCaching(DcMotor.RunMode runMode, DcMotor.RunMode prevRunMode, DcMotor motor)
    {
        if (runMode != prevRunMode)
        {
            motor.setMode(runMode);
            prevRunMode = runMode;
        }
        return prevRunMode;
    }

    public static MathResult mathCaching(DoubleUnaryOperator math, double currentInput, double prevInput, double prevResult)
    {
        if (Math.abs(currentInput - prevInput) > EPSILON_DELTA)
        {

            prevInput = currentInput;
            prevResult = applyOperation(currentInput, math);

        }
        // i have no idea why this work, to many lambdas for my brain
        return new MathResult(prevInput, prevResult);
    }

    public static double applyOperation(double x, DoubleUnaryOperator operator)
    {
        return operator.applyAsDouble(x);
    }

    /**
     *
     * @param hubs the list of all hubs
     * @return returns a new list where chub is always at index 0
     */
    public static List<LynxModule> setHubs(List<LynxModule> hubs)
    {
        if (hubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(hubs.get(0).getSerialNumber()))
        {
            chub = hubs.get(0);
            expHub = hubs.get(1);
        } else
        {
            chub = hubs.get(1);
            expHub = hubs.get(0);
        }
        hubs.clear();
        hubs.add(chub);
        hubs.add(expHub);
        return hubs;
    }


    // This is here because multiple pipelines access it
    public static Tags setSeenAprilTags(double tagsNum, double[] tags)
    {
        return new Tags(tagsNum, tags);

    }

    public static double inchesToTicksRailCorrected(double in)
    {
        return in / 25.207 * -1;
    }

}


