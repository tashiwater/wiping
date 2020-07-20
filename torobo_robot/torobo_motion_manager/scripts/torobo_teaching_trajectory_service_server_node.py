#!/usr/bin/env python
import rospy
import numpy
from trajectory_msgs.msg import JointTrajectoryPoint
from torobo_msgs.srv import DeleteTeachingTrajectory
from torobo_msgs.srv import DeleteTeachingTrajectoryResponse
from torobo_msgs.srv import GetTeachingTrajectory
from torobo_msgs.srv import GetTeachingTrajectoryResponse
from torobo_msgs.srv import GetTeachingTrajectoryNames
from torobo_msgs.srv import GetTeachingTrajectoryNamesResponse
from torobo_msgs.srv import RecordTeachingTrajectory
from torobo_msgs.srv import RecordTeachingTrajectoryResponse

class TeachingTrajectoryServiceServer(object):
    def __init__(self):
        self._recordServiceServer = rospy.Service('record_teaching_trajectory', RecordTeachingTrajectory, self.RecordService)
        self._deleteServiceServer = rospy.Service('delete_teaching_trajectory', DeleteTeachingTrajectory, self.DeleteService)
        self._getServiceServer = rospy.Service('get_teaching_trajectory', GetTeachingTrajectory, self.GetService)
        self._getNamesServiceServer = rospy.Service('get_teaching_trajectory_names', GetTeachingTrajectoryNames, self.GetNamesService)

    def RecordService(self, req):
        traj = []
        for p in req.trajectory.points:
            point = {}
            point["time_from_start"] = p.time_from_start.to_sec()
            point["positions"] = p.positions
            point["velocities"] = p.velocities
            point["accelerations"] = p.accelerations
            point["effort"] = p.effort
            traj.append(point)
        paramName = rospy.get_namespace() + "teaching_trajectories/" + req.teachingTrajectoryName
        nameParamName = rospy.get_namespace() + "teaching_trajectories/names"

        names = self.GetNames()
        if names is None:
            names = []
        names.append(req.teachingTrajectoryName)
        # duplicate cut
        names = list(set(names))

        rospy.set_param(paramName, traj)
        rospy.set_param(nameParamName, names)

        res = RecordTeachingTrajectoryResponse()
        res.success = True
        return res

    def DeleteService(self, req):
        res = DeleteTeachingTrajectoryResponse()
        paramName = rospy.get_namespace() + "teaching_trajectories/" + req.teachingTrajectoryName
        nameParamName = rospy.get_namespace() + "teaching_trajectories/names"
        traj = rospy.get_param(paramName, None)
        if traj is None:
            res.success = False
            return True
        rospy.delete_param(paramName)

        # delete name from teaching_trajectories/names
        names = self.GetNames()
        names.remove(req.teachingTrajectoryName)
        rospy.set_param(names)
        res.success = True
        return True

    def GetService(self, req):
        res = GetTeachingTrajectoryResponse()
        paramName = rospy.get_namespace() + "teaching_trajectories/" + req.teachingTrajectoryName
        trajectory = rospy.get_param(paramName, None)
        if trajectory is None:
            res.success = False
        else:
            points = []
            for p in trajectory:
                point = JointTrajectoryPoint()
                point.positions = p["positions"]
                point.velocities = p["velocities"]
                point.accelerations = p["accelerations"]
                point.effort = p["effort"]
                point.time_from_start = rospy.Duration.from_sec(p["time_from_start"])
                points.append(point)
            res.trajectory.header.stamp = rospy.Time.now()
            for i in range(len(points[0].positions)):
                res.trajectory.joint_names.append("joint_" + str(i+1))
            res.trajectory.points = points
            res.success = True
        return res

    def GetNamesService(self, req):
        res = GetTeachingTrajectoryNamesResponse()
        names = self.GetNames()
        if names is None:
            res.success = False
        else:
            res.success = True
        return res
    
    def GetNames(self):
        paramName = rospy.get_namespace() + "teaching_trajectories/names"
        names = rospy.get_param(paramName, None)
        return names

if __name__ == '__main__':
    rospy.init_node('teaching_trajectory_service_server_node')
    rospy.loginfo('teaching_trajectory_service_server_node')
    try:
        server = TeachingTrajectoryServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass