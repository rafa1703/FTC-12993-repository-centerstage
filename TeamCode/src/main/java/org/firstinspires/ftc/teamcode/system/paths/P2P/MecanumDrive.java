package org.firstinspires.ftc.teamcode.system.paths.P2P;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.angleWrap;
import static org.firstinspires.ftc.teamcode.system.hardware.Globals.normalizeRadians;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CrossTrackError.calculateCTE;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CrossTrackError.calculateLookahead;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions.retractVector;

import android.os.LocaleList;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.accessory.PID;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.Globals;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.CurvePoint;
import org.firstinspires.ftc.teamcode.system.paths.PurePersuit.MathFunctions;
import org.firstinspires.ftc.teamcode.system.paths.splines.Trajectory;
import org.opencv.core.Point;

import java.util.ArrayList;

public class MecanumDrive
{
    public static boolean ENABLED = true;


    public enum RunMode
    {
        PID,
        P2P,
        Vector,
        PP,
        GVF
    }

    public static PIDController TRANSLATIONAL_PID = new PIDController(0.27, 0.00000, 0.00034);
    public static PIDController HEADING_PID = new PIDController(0.5, 0.008, 0.00034);
    private DcMotor FL, FR, BL, BR; // TODO: hardware class > then this
    private RunMode runMode;
    private Localizer localizer;
    public Vector powerVector = new Vector();
    private Pose targetPose = new Pose();
    public Vector targetVector = new Vector();

    private static double ks = 0.03;
    public double lateralMultiplier = 1.08; //= 1.1194029851;
    public static double headingMultiplier = 1;
    private double overallMultiplier = 1;

    private final double velocityThreshold = 1;

    private TimedSupplier<Double> voltageSupplier;

    public MecanumDrive(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, RunMode runMode, TimedSupplier<Double> supplier)
    {
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;

        this.runMode = runMode;

        this.voltageSupplier = supplier;
    }


    private void updatePowerVector()
    {
        switch (runMode)
        {
            case PID:
                // This is gonna be a standard pid, probably never will be used but so like maybe rename to position lock
                // driveToPosition(0, 0, 0, null);
                driveToPosition(targetPose.getX(), targetPose.getY(), targetPose.getHeading(), localizer.getPredictedPoseEstimate().toPose2d());
                break;
            case P2P:
                P2P();
                break;
            case Vector:
                powerVector = new Vector(targetVector);
                powerVector = Vector.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case PP:
                /*PP();*/
                break;
        }
        if (runMode == RunMode.P2P || runMode == RunMode.PP)
        {
            if (Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
                powerVector.scaleToMagnitude(1);
            powerVector.scaleBy(overallMultiplier); // search for the speed controller lol
        }
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading, Pose2d poseEstimate)
    {
        /*double robotX = poseEstimate.getX();
        double robotY = poseEstimate.getY();
        double robotTheta = poseEstimate.getHeading();
        double x = TRANSLATIONAL_PID.calculate(robotX, targetX);
        double y = -TRANSLATIONAL_PID.calculate(robotY, targetY);
        double theta = -HEADING_PID.calculate(AngleWrap(targetHeading - robotTheta));


        double x_rotated = (x * Math.cos(robotTheta) - y * Math.sin(robotTheta));
        double y_rotated = (x * Math.sin(robotTheta) + y * Math.cos(robotTheta));

        double FL = MathUtils.clamp(x_rotated + y_rotated + theta, -1, 1);
        double BL = MathUtils.clamp(x_rotated - y_rotated + theta, -1, 1);
        double FR = MathUtils.clamp(x_rotated - y_rotated - theta, -1, 1);
        double BR = MathUtils.clamp(x_rotated + y_rotated - theta, -1, 1);


        this.FL.setPower(FL);
        this.BL.setPower(BL);
        this.FR.setPower(FR);
        this.BR.setPower(BR);*/
    }

    private void P2P()
    {
        Pose currentPose = localizer.getPredictedPoseEstimate();

        double xDiff = targetPose.getX() - currentPose.getX();
        double yDiff = targetPose.getY() - currentPose.getY();

        double distance = Math.hypot(xDiff, yDiff);

        double calculatedCos = xDiff / distance;
        double calculatedSin = yDiff / distance;

        double translationalPower = TRANSLATIONAL_PID.calculate(0, distance);

        powerVector = new Vector(translationalPower * calculatedCos, translationalPower * calculatedSin);
        powerVector = Vector.rotateBy(powerVector, currentPose.getHeading());

        double headingDiff = angleWrap(targetPose.getHeading() - currentPose.getHeading());

        double headingPower = HEADING_PID.calculate(0, headingDiff) * headingMultiplier;

        // at heading 0 and 180 the y component is lateral, and at 90 and 270  the x is the lateral
        // so y * cos(H) * LateralMulti and x * sin(H) * LateralMulti deals with that
        //double xPow = powerVector.getX() * Math.sin(currentPose.getHeading()) * lateralMultiplier;
        //double yPow = powerVector.getY() * Math.cos(currentPose.getHeading()) * lateralMultiplier;

        powerVector = new Vector(powerVector.getX(), powerVector.getY() * lateralMultiplier, headingPower);
    }

    public double FLPower, FRPower, BLPower, BRPower;

    private void updateMotors()
    {
        if (runMode == RunMode.P2P)
        {

            double actualKs = ks * 12.0 / voltageSupplier.get();

            FLPower = (powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ());
            FL.setPower(FLPower);

            FRPower = (powerVector.getX()) + powerVector.getY() + powerVector.getZ() * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ());
            FR.setPower(FRPower);

            BLPower = (powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ());
            BL.setPower(BLPower);

            BRPower = (powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs)
                    + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ());
            BR.setPower(BRPower);

        } else if (runMode == RunMode.GVF)
        {


        } else if(runMode == RunMode.Vector)
        {
            double actualKs = ks * 12.0 / voltageSupplier.get();

            FL.setPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
            FR.setPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
            BL.setPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));
            BR.setPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
        }
    }

    public void update()
    {
        if (!ENABLED) return;
        localizer.update();
        updatePowerVector();
        updateMotors();
    }

    // TODO: this should absolutely be cached and not done like this
    public double getVoltage()
    {
        return voltageSupplier.get();
    }

    public void setTargetPose(Pose pose)
    {
        this.targetPose = pose;
    }

    public void setTargetPose(Pose2d pose)
    {
        this.targetPose = new Pose(pose);
    }

    public void setTargetVector(Vector Vector)
    {
        this.targetVector = Vector;
    }

    /*public void setTargetPath(ArrayList<CurvePoint> path)
    {
        currentPath = path;
    }*/

    public RunMode getRunMode()
    {
        return runMode;
    }

    public Localizer getLocalizer()
    {
        return localizer;
    }

    public void setLocalizer(Localizer localizer)
    {
        this.localizer = localizer;
    }

    public Pose getTargetPose()
    {
        return targetPose;
    }

    public void setRunMode(RunMode runMode)
    {
        this.runMode = runMode;
    }

    public boolean reachedTarget(double tolerance)
    {
        if (runMode == RunMode.Vector) return false;
        /*if (runMode == RunMode.PP)
            return localizer.getPoseEstimate().getDistance(currentPath.get(currentPath.size() - 1).toPose()) <= tolerance;*/
        return localizer.getPoseEstimate().getDistance(targetPose) <= tolerance;
    }

    public boolean reachedHeading(double tolerance)
    {
        if (runMode == RunMode.Vector) return false; // for now PP will be caught here
        if (runMode == RunMode.PP) return false;
        return Math.abs(normalizeRadians(targetPose.getHeading() - localizer.getHeading())) <= tolerance;
    }

    public boolean stopped()
    {
        return localizer.getVelocity().getMagnitude() <= velocityThreshold;
    }

    public void setSpeed(double speed)
    {
        MathUtils.clamp(speed, 0, 1);
        overallMultiplier = speed;
    }

    public void followTrajectory(@NonNull Trajectory trajectory)
    {
        runMode = RunMode.Vector;
        Pose currentPose = localizer.getPredictedPoseEstimate();
        setTargetVector(trajectory.getPowerVector(currentPose));
        if (trajectory.usePid())
        {
            runMode = RunMode.P2P;
            setTargetPose(trajectory.getFinalPose());
        }
    }
    public void followTrajectoryTangentially(@NonNull Trajectory trajectory, boolean reverse)
    {
        runMode = RunMode.Vector;
        Pose currentPose = localizer.getPredictedPoseEstimate();
        setTargetVector(trajectory.getTangentPowerVector(currentPose, reverse));
        if (trajectory.usePid())
        {
            runMode = RunMode.P2P;
            setTargetPose(trajectory.getFinalPose());
        }
    }






    

















































    // this is PP crap, doesn't work
    // PP

    /*
    public CurvePoint currentPoint;
    public  CurvePoint lastPoint;
    public  boolean finished = false;
    public  double lookAheadDis;
    public ArrayList<CurvePoint> currentPath;
    private void PP()
    {
        Pose currentPose = localizer.getPredictedPoseEstimate();

        followCurve(currentPath, Math.toRadians(90), currentPose, currentPose.getHeading());

    }




    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle, Pose currentPose, double heading){
        if (lookAheadDis == 0) // the initial one should always be the first
        {
            lookAheadDis = allPoints.get(0).followDistance;
        }
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(currentPose.getX(), currentPose.getY()),
                lookAheadDis, currentPose);


        lookAheadDis = calculateLookahead(calculateCTE(currentPose.toPoint(), followMe.toPoint(), lookAheadDis, heading), 1, 5, 12);


        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, currentPose);
   *//*
        if (finished)
        {
            CurvePoint startPoint = allPoints.get(allPoints.size() -2);
            CurvePoint endPoint = allPoints.get(allPoints.size() - 1);
            CurvePoint finalPoint = extendVector(startPoint, endPoint);

            goToHeading(finalPoint.x, finalPoint.y, finalPoint.moveSpeed, followAngle, finalPoint.turnSpeed);
        }

         *//*

        lastPoint = currentPoint;

        currentPoint = followMe;

    }

    public CurvePoint getFollowPointPath (ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius, Pose currentPosition){
        CurvePoint followMe;
        //this makes the robot follow the previous point followed

        if (lastPoint != null)
        {
            followMe = lastPoint;
        }
        else // if at the intersect
        {
            followMe = new CurvePoint(pathPoint.get(0));
        }


        for (int i = 0; i < pathPoint.size() - 1; i ++)
        {
            CurvePoint startLine = pathPoint.get(i);
            CurvePoint endLine = pathPoint.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),
                    endLine.toPoint());


            double closestAngle = Double.MAX_VALUE;

            for (Point thisIntersection : intersections)
            {
                double angle = Math.atan2(thisIntersection.y - currentPosition.getY(), thisIntersection.x - currentPosition.getX());
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - currentPosition.getHeading()));

                if (deltaAngle < closestAngle)
                {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection); //TODO: this only sets the x, y. The other parameters are preserved from the curvepoint declaration


                    *//*
                    CurvePoint finalStartLine = pathPoint.get(pathPoint.size() - 2);
                    CurvePoint finalEndLine = pathPoint.get(pathPoint.size() - 1);
                    finalStartLine = retractVector(finalStartLine, finalEndLine);

                    ArrayList<Point> interFinal = lineCircleIntersection(thisIntersection,0.2, finalStartLine.toPoint(), finalEndLine.toPoint());
                    if (interFinal.size() > 0)
                    {
                        finished = true;
                    }*//*

                }
            }
            if (currentPoint != null)
            {
                double distanceFromAFollowMe = Math.sqrt(Math.pow(followMe.x - startLine.x, 2) + Math.pow(followMe.y - startLine.x, 2));
                double distanceFromACurrentPoint = Math.sqrt(Math.pow(currentPoint.x - startLine.x, 2) + Math.pow(currentPoint.y - startLine.x, 2));

                if (distanceFromACurrentPoint > distanceFromAFollowMe)
                {
                    followMe.setPoint(currentPoint.toPoint());
                }
            }
        }
        *//*if (finished)
        {
            followMe = new CurvePoint(pathPoint.get(pathPoint.size()-1));
        }*//*

        return followMe;
    }

  public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, @NonNull Pose currentPosition) {


        double distanceToTarget = Math.hypot(x-currentPosition.getX(), y-currentPosition.getY());

        double absoluteAngleToTarget = Math.atan2(y-currentPosition.getY(), x-currentPosition.getX());

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (currentPosition.getHeading() - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double denominator = (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementXPower = relativeXToPoint / denominator;
        double movementYPower = relativeYToPoint / denominator;

        powerVector = new Vector(movementXPower * movementSpeed, movementYPower * movementSpeed);

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double headingPower = relativeTurnAngle/Math.toRadians(30) * turnSpeed;

        if (distanceToTarget < 1 ){
            headingPower = 0;
        }

        powerVector = new Vector(powerVector.getX(), powerVector.getY(), headingPower);
    }

    private void updatePPmotors()
    {
        double strafeSpeed = powerVector.getY();
        double forwardSpeed = powerVector.getX();
        double turnSpeed = powerVector.getZ();

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        Pose currentPose = localizer.getPredictedPoseEstimate();
        input = input.rotateBy(-currentPose.getHeading());

        double distance = Math.hypot(strafeSpeed, forwardSpeed);
        *//*
        // trying to counter act no pid
        if (distance < 0.5)
        {
            forwardSpeed = 0;
            strafeSpeed = 0;
        }*//*

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[0] += turnSpeed;
        wheelSpeeds[1] -= turnSpeed;
        wheelSpeeds[2] += turnSpeed;
        wheelSpeeds[3] -= turnSpeed;

        normalize(wheelSpeeds); // normalize for like

    }

    private void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }*/


}
