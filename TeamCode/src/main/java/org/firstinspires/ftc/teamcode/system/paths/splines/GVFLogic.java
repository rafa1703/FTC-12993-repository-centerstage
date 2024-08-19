package org.firstinspires.ftc.teamcode.system.paths.splines;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.normalizeRadians;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;
import org.firstinspires.ftc.teamcode.system.paths.P2P.Vector;
import org.opencv.core.Point;

public class GVFLogic
{

    //This whole
    public boolean followTangentially = false;
    public boolean reverse = false;
    public boolean splineHeading = false;
    public boolean usePID = false;

    public Vector calculate(@NonNull BezierCurve curve, Pose pose, boolean slowDown, double maxSpeed)
    {
        // correction is like the go back to the fucking line and then path shit is follow the bitch spline
        Point robot = pose.toPoint();
        Vector endPoint = curve.getEndPoint();
        // FIXME: 18/8/2024 i am an idiot and this will proably never fucking work and i just wasting time because i m...
        double t = curve.returnClosestT(robot);
        Vector closestPoint = new Vector(curve.parametric(t));
        Vector derivative = curve.getTangentialVector(t);
        Vector robotToPoint = closestPoint.subtract(robot);
        Vector robotToEnd = endPoint.subtract(robot);

        double CORRECTION_DIS = 30; // tune
        double SAVING_THROW_DIS = 24;

        double directPursuitThreshold = 1;
        // going from above now should not break anything and improve cycle times
        for(double i = 1; i >= 0; i -= 1 / curve.getInterval()) // okay so this is like at the point lets iterate trough all points and find out if the distance of the point to the end point if its lower than the threshold
        {
            double dist = endPoint.subtract(curve.parametric(i)).getMagnitude();
            // if we are closer than the threshold we can start directly following the curve
            if (dist < SAVING_THROW_DIS)
            {
                directPursuitThreshold = i;
                break;
            }
        }

        double correctionScale = Math.min(1, robotToPoint.getMagnitude() / CORRECTION_DIS);
        double direction = headingInterpolation(derivative.getAngle(), robotToPoint.getAngle(), correctionScale);

        Vector tempRobot = new Vector(robot.x, robot.y);
        // this just say if the closest point and the robot is tangent or t is greater than the follow the path threshold we follow the end point
        if ((t == 1 && Math.abs(tempRobot.subtract(closestPoint).getAngle() - derivative.getAngle()) <= 0.5 * Math.PI)
                || t >= directPursuitThreshold)
        {
            direction = endPoint.subtract(robot).getAngle();
        }

        Vector movementVector = new Vector(Math.cos(direction), Math.sin(direction));
        double speed = maxSpeed;

        if (robotToEnd.getMagnitude() < 34 && slowDown)
        {
            // like a weighted average for the speed
            speed = interpolation(0.15, speed, robotToEnd.getMagnitude() / 34);
        }
        movementVector.scaleBy(speed);

        if (followTangentially && !splineHeading) // reverse only affects if we following tangentially so it can be nested here
        {
            if(t == 1 && slowDown) // we only slowdown on the last curve
            {
                derivative = curve.getTangentialVector(1 - (1/ curve.getInterval())); // this is because at 1 the derivative is NaN
            }
            double angle = derivative.getAngle();
            if (reverse) angle += Math.toRadians(180);
            double headingScale = Math.abs(Math.min(normalizeRadians(angle - pose.getHeading()) / Math.toRadians(70), 1)); // rn i think 70 is the best angle to consider max thing
            double headingDiff = headingInterpolation(pose.getHeading(), angle, headingScale) - pose.getHeading();  //Math.min(normalizeRadians(derivative.getAngle() - pose.getHeading()) / Math.toRadians(30), 1); // Who the fuck knows if this is gonna work
            // we want to remove the current heading because interpolation at t = 0 returns the current heading
            movementVector = new Vector(movementVector.getX(), movementVector.getY(), headingDiff);

        }
        // tbh idk if the dist check is necessary here
        usePID = t == 1 && robotToEnd.getMagnitude() < 4;
        return movementVector;

    }

    public double headingInterpolation(double theta1, double theta2, double t)
    {
        //Normalize radians with a scaled (t)
        double diff = theta2 - theta1;
        diff %= 2 * Math.PI;
        if (Math.abs(diff) > Math.PI) {
            if (diff > 0) {
                diff -= 2 * Math.PI;
            } else {
                diff += 2 * Math.PI;
            }
        }
        return theta1 + t * diff;
    }

    private double interpolation(double p1, double p2, double t) {
        return (1 - t) * p1 + t * p2;
    }


    private double binomalSearch(BezierCurve curve, Point robot, double start, double end)
    {
        double middle = (start + end) / 2;

        Point startPoint = curve.parametric(start);
        Point endPoint = curve.parametric(end);
        Vector robotToEnd = new Vector(endPoint.x - robot.x, endPoint.y - robot.y);
        Vector robotToStart = new Vector(startPoint.x - robot.x, startPoint.y - robot.y);

        if (robotToStart.getMagnitude() < robotToEnd.getMagnitude())
        {
            return binomalSearch(curve, robot, start, middle);
        }
        else if (robotToStart.getMagnitude() > robotToEnd.getMagnitude())
        {
            return binomalSearch(curve, robot, middle, end);
        }
        else return middle;
        //if ()

    }

        private double binomalSearch2(BezierCurve curve, Point point, double start, double end)
    {
        // is this just like not the same ans iterating trough the whole fucking curve??
        Vector robot = new Vector(point);
        Point startPoint = curve.parametric(start);
        Point endPoint = curve.parametric(end);
        Vector robotToEnd = new Vector(endPoint.x - robot.getX(), endPoint.y - robot.getY());
        Vector robotToStart = new Vector(startPoint.x - robot.getX(), startPoint.y - robot.getY());

        if (robotToStart.getMagnitude() < robotToEnd.getMagnitude())
        {
            return binomalSearch(curve, point, start, end - (1/curve.getInterval()));
        }
        else if (robotToStart.getMagnitude() > robotToEnd.getMagnitude())
        {
            return binomalSearch(curve, point, start +(1/ curve.getInterval()), end);
        }
        return start;

    }

    public void setFollowTangentially(boolean followTangentially)
    {
        this.followTangentially = followTangentially;
    }

    public void setSplineHeading(boolean splineHeading)
    {
        this.splineHeading = splineHeading;
    }

    public void setReverse(boolean reverse)
    {
        this.reverse = reverse;
    }

    public boolean isFollowTangentially()
    {
        return followTangentially;
    }

    public boolean isReverse()
    {
        return reverse;
    }

    public boolean usePID()
    {
        return usePID;
    }
}
