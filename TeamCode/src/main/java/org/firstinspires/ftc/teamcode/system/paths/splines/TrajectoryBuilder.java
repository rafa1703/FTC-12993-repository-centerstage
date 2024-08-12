package org.firstinspires.ftc.teamcode.system.paths.splines;

import java.util.ArrayList;

public class TrajectoryBuilder
{
    private final ArrayList<TrajectorySegment> segments = new ArrayList<>();

    public TrajectoryBuilder(TrajectorySegment segment){
        segments.add(segment);
    }

    public TrajectoryBuilder addSegment(TrajectorySegment segment){
        segments.add(segment);
        return this;
    }

    public Trajectory build(){
        return new Trajectory(segments);
    }
}
