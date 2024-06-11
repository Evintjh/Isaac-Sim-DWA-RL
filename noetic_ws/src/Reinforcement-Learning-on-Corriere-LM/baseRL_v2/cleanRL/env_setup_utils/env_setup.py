
def setup_action_graph(num_envs):
    import omni.graph.core as og
    nodes_to_create = []
    values_to_set = []
    nodes_to_connect =[]
    nodes_to_create.append(("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"))
    nodes_to_create.append(("on_playback_tick", "omni.graph.action.OnPlaybackTick"))

    for i in range(num_envs):    
        nodes_to_create.append((f"isaac_read_lidar_beams_node_{i}","omni.isaac.range_sensor.IsaacReadLidarBeams"))
        nodes_to_create.append((f"ros1_publish_laser_scan_{i}","omni.isaac.ros_bridge.ROS1PublishLaserScan"))
        nodes_to_create.append((f"add_relationship_node_{i}","omni.graph.action.AddPrimRelationship"))

        values_to_set.append((f"add_relationship_node_{i}.inputs:name","inputs:lidarPrim"))
        values_to_set.append((f"add_relationship_node_{i}.inputs:path",f"/ActionGraph/isaac_read_lidar_beams_node_{i}"))
        values_to_set.append((f"add_relationship_node_{i}.inputs:target",f"/envs/env_{i}/jackal/base_link/sick_lms1xx_lidar_frame/Lidar"))
        values_to_set.append((f"ros1_publish_laser_scan_{i}.inputs:topicName",f"/laser_scan_{i}"))

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
        

    print(nodes_to_create)
    (graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
        og.Controller.Keys.CREATE_NODES: nodes_to_create,
        og.Controller.Keys.SET_VALUES: values_to_set,
        og.Controller.Keys.CONNECT: nodes_to_connect
        
        }
    )