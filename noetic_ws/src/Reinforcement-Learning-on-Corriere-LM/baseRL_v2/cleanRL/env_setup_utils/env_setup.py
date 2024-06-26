
# def setup_action_graph(num_envs):
#     import omni.graph.core as og
#     nodes_to_create = []
#     values_to_set = []
#     nodes_to_connect =[]
#     nodes_to_create.append(("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"))
#     nodes_to_create.append(("on_playback_tick", "omni.graph.action.OnPlaybackTick"))

#     for i in range(num_envs):    
#         nodes_to_create.append((f"isaac_read_lidar_beams_node_{i}","omni.isaac.range_sensor.IsaacReadLidarBeams"))
#         nodes_to_create.append((f"ros1_publish_laser_scan_{i}","omni.isaac.ros_bridge.ROS1PublishLaserScan"))
#         nodes_to_create.append((f"add_relationship_node_{i}","omni.graph.action.AddPrimRelationship"))

#         values_to_set.append((f"add_relationship_node_{i}.inputs:name","inputs:lidarPrim"))
#         values_to_set.append((f"add_relationship_node_{i}.inputs:path",f"/ActionGraph/isaac_read_lidar_beams_node_{i}"))
#         values_to_set.append((f"add_relationship_node_{i}.inputs:target",f"/envs/env_{i}/jackal/base_link/sick_lms1xx_lidar_frame/Lidar"))
#         values_to_set.append((f"ros1_publish_laser_scan_{i}.inputs:topicName",f"/laser_scan_{i}"))

#         nodes_to_connect.append((f"isaac_read_simulation_time.outputs:simulationTime",f"ros1_publish_laser_scan_{i}.inputs:timeStamp"))
#         nodes_to_connect.append(("on_playback_tick.outputs:tick", f"isaac_read_lidar_beams_node_{i}.inputs:execIn"))
#         nodes_to_connect.append(("on_playback_tick.outputs:tick", f"add_relationship_node_{i}.inputs:execIn"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:azimuthRange",f"ros1_publish_laser_scan_{i}.inputs:azimuthRange"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:depthRange",f"ros1_publish_laser_scan_{i}.inputs:depthRange"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:execOut",f"ros1_publish_laser_scan_{i}.inputs:execIn"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:horizontalFov",f"ros1_publish_laser_scan_{i}.inputs:horizontalFov"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:horizontalResolution",f"ros1_publish_laser_scan_{i}.inputs:horizontalResolution"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:intensitiesData",f"ros1_publish_laser_scan_{i}.inputs:intensitiesData"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:linearDepthData",f"ros1_publish_laser_scan_{i}.inputs:linearDepthData"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:numCols",f"ros1_publish_laser_scan_{i}.inputs:numCols"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:numRows",f"ros1_publish_laser_scan_{i}.inputs:numRows"))
#         nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:rotationRate",f"ros1_publish_laser_scan_{i}.inputs:rotationRate"))
        

#     print(nodes_to_create)
#     (graph, _, _, _) = og.Controller.edit(
#         {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
#         {
#         og.Controller.Keys.CREATE_NODES: nodes_to_create,
#         og.Controller.Keys.SET_VALUES: values_to_set,
#         og.Controller.Keys.CONNECT: nodes_to_connect
        
#         }
#     )



def setup_action_graph(num_envs):
    import omni.graph.core as og

    # creates a list to feed into og class
    nodes_to_create = []
    values_to_set = []
    nodes_to_connect =[]

    # list appends a tuple of strings
    nodes_to_create.append(("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"))
    nodes_to_create.append(("on_playback_tick", "omni.graph.action.OnPlaybackTick"))
    nodes_to_create.append(("ClockPublisher", "omni.isaac.ros_bridge.ROS1PublishClock"))


    # list appends a tuple of strings
    for i in range(num_envs):    
        nodes_to_create.append((f"isaac_read_lidar_beams_node_{i}","omni.isaac.range_sensor.IsaacReadLidarBeams"))
        nodes_to_create.append((f"ros1_publish_laser_scan_{i}","omni.isaac.ros_bridge.ROS1PublishLaserScan"))
        nodes_to_create.append((f"add_relationship_node_{i}","omni.graph.action.AddPrimRelationship"))
        nodes_to_create.append((f"ComputeOdom{i}", "omni.isaac.core_nodes.IsaacComputeOdometry"))        
        nodes_to_create.append((f"PublishOdometry{i}", "omni.isaac.ros_bridge.ROS1PublishOdometry")) 
        # nodes_to_create.append((f"PublishOdometryTfXform{i}", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"))
        # nodes_to_create.append((f"tf_baselink_lidar{i}", "omni.isaac.ros_bridge.ROS1PublishTransformTree"))
        # # nodes_to_create.append((f"tf_midmount_down{i}", "omni.isaac.ros_bridge.ROS1PublishTransformTree"))
        # # nodes_to_create.append((f"tf_frontmount_lidar{i}", "omni.isaac.ros_bridge.ROS1PublishTransformTree"))
        # nodes_to_create.append((f"tf_chassis_down{i}", "omni.isaac.ros_bridge.ROS1PublishTransformTree"))

        values_to_set.append((f"add_relationship_node_{i}.inputs:name","inputs:lidarPrim"))
        values_to_set.append((f"add_relationship_node_{i}.inputs:path",f"/ActionGraph/isaac_read_lidar_beams_node_{i}"))
        values_to_set.append((f"add_relationship_node_{i}.inputs:target",f"/envs/env_{i}/jackal/base_link/sick_lms1xx_lidar_frame/Lidar"))
        values_to_set.append((f"ros1_publish_laser_scan_{i}.inputs:topicName",f"/laser_scan_{i}"))
        values_to_set.append((f"PublishOdometry{i}.inputs:topicName", f"/odom{i}")) 
        values_to_set.append(f"ros1_publish_laser_scan_{i}.inputs:frameId", f"Lidar{i}")
        # values_to_set.append(f"PublishOdometryTfXform{i}.inputs:parentFrameId", f"/odom{i}")
        # values_to_set.append(f"PublishOdometryTfXform{i}.inputs:childFrameId", f"/base_link{i}")
        values_to_set.append(f"ComputeOdom{i}.inputs:chassisPrim", f"/envs/env_{i}/jackal")
        # values_to_set.append(f"tf_baselink_lidar{i}.inputs:parentPrim", f"/envs/env_{i}/jackal/base_link")
        # values_to_set.append(f"tf_baselink_lidar{i}.inputs:targetPrims", f"/envs/env_{i}/jackal/sick_lms1xx_lidar_frame/Lidar")
        # # values_to_set.append(f"tf_midmount_down{i}.inputs:parentPrim", "/jackal/mid_mount")
        # # values_to_set.append(f"tf_midmount_down{i}.inputs:targetPrims", ["/jackal/front_mount","/jackal/rear_mount"])
        # # values_to_set.append(f"tf_frontmount_lidar{i}.inputs:parentPrim", "/jackal/front_mount")
        # # values_to_set.append(f"tf_frontmount_lidar{i}.inputs:targetPrims", "/jackal/front_mount/Lidar")
        # values_to_set.append(f"tf_chassis_down{i}.inputs:parentPrim", f"/envs/env_{i}/jackal/base_link")
        # values_to_set.append(f"tf_chassis_down{i}.inputs:targetPrims", [
        #     f"/envs/env_{i}/jackal/front_left_wheel_link",f"/envs/env_{i}/jackal/front_right_wheel_link",
        #     f"/envs/env_{i}/jackal/rear_left_wheel_link",f"/envs/env_{i}/jackal/rear_right_wheel_link"
        # ])

        nodes_to_connect.append((f"isaac_read_simulation_time.outputs:simulationTime",f"ros1_publish_laser_scan_{i}.inputs:timeStamp"))
        nodes_to_connect.append(("on_playback_tick.outputs:tick", f"isaac_read_lidar_beams_node_{i}.inputs:execIn"))
        nodes_to_connect.append(("on_playback_tick.outputs:tick", f"add_relationship_node_{i}.inputs:execIn"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:azimuthRange",f"ros1_publish_laser_scan_{i}.inputs:azimuthRange"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:depthRange",f"ros1_publish_laser_scan_{i}.inputs:depthRange"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:execOut",f"ros1_publish_laser_scan_{i}.inputs:execIn"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:horizontalFov",f"ros1_publish_laser_scan_{i}.inputs:horizontalFov"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:horizontalResolution",f"ros1_publish_laser_scan_{i}.inputs:horizontalResolution"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:intensitiesData",f"ros1_publish_laser_scan_{i}.inputs:intensitiesData"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:linearDepthData",f"ros1_publish_laser_scan_{i}.inputs:linearDepthData"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:numCols",f"ros1_publish_laser_scan_{i}.inputs:numCols"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:numRows",f"ros1_publish_laser_scan_{i}.inputs:numRows"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:rotationRate",f"ros1_publish_laser_scan_{i}.inputs:rotationRate"))
        nodes_to_connect.append((f"isaac_read_lidar_beams_node_{i}.outputs:rotationRate",f"ros1_publish_laser_scan_{i}.inputs:rotationRate"))
        # nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"tf_baselink_chassis{i}.inputs:execIn"))
        # nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"tf_baselink_chassis{i}.inputs:timeStamp"))
        # # nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"tf_midmount_down{i}.inputs:execIn"))
        # # nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"tf_midmount_down{i}.inputs:timeStamp"))
        # # nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"tf_frontmount_lidar{i}.inputs:execIn"))
        # # nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"tf_frontmount_lidar{i}.inputs:timeStamp"))
        # nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"tf_chassis_down{i}.inputs:execIn"))
        # nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"tf_chassis_down{i}.inputs:timeStamp"))
        nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"ComputeOdom{i}.inputs:execIn"))
        nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"PublishOdometry{i}.inputs:execIn"))
        # nodes_to_connect.append(("OnPlaybackTick.outputs:tick", f"PublishOdometryTfXform{i}.inputs:execIn"))
        nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"PublishOdometry{i}.inputs:timeStamp"))
        # nodes_to_connect.append(("ReadSimTime.outputs:simulationTime", f"PublishOdometryTfXform{i}.inputs:timeStamp"))
        nodes_to_connect.append((f"ComputeOdom{i}.outputs:angularVelocity", f"PublishOdometry{i}.inputs:angularVelocity"))
        nodes_to_connect.append((f"ComputeOdom{i}.outputs:linearVelocity", f"PublishOdometry{i}.inputs:linearVelocity"))
        nodes_to_connect.append((f"ComputeOdom{i}.outputs:orientation", f"PublishOdometry{i}.inputs:orientation"))
        nodes_to_connect.append((f"ComputeOdom{i}.outputs:position", f"PublishOdometry{i}.inputs:position"))
        # nodes_to_connect.append((f"ComputeOdom{i}.outputs:orientation", f"PublishOdometryTfXform{i}.inputs:rotation"))
        # nodes_to_connect.append((f"ComputeOdom{i}.outputs:position", f"PublishOdometryTfXform{i}.inputs:translation"))

    print(nodes_to_create)
    (graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
        og.Controller.Keys.CREATE_NODES: nodes_to_create,
        og.Controller.Keys.SET_VALUES: values_to_set,
        og.Controller.Keys.CONNECT: nodes_to_connect
        
        }
    )