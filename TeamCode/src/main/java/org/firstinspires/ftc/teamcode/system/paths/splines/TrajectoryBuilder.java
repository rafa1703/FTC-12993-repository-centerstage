package org.firstinspires.ftc.teamcode.system.paths.splines;

import org.firstinspires.ftc.teamcode.system.paths.P2P.Pose;

import java.util.ArrayList;

public class TrajectoryBuilder
{
    private final ArrayList<TrajectorySegment> segments = new ArrayList<>();
    private Pose finalPose = null;

    public TrajectoryBuilder(TrajectorySegment segment){
        segments.add(segment);
    }

    public TrajectoryBuilder addSegment(TrajectorySegment segment){
        segments.add(segment);
        return this;
    }
    public TrajectoryBuilder addFinalPose(Pose pose)
    {
        finalPose = pose;
        return this;
    }

    public Trajectory build(){
        return new Trajectory(segments);
    }
    public Trajectory end() {return new Trajectory(segments, finalPose);}
}
