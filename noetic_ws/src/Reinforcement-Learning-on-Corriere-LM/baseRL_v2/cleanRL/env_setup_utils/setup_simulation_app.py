def setup_simulation_app():
    CONFIG = {
            # "width": 1280,
            # "height": 720,
            "width": 1920,
            "height": 1080,
            # "window_width": 1920,
            # "window_height": 1080,
            "window_width": 2560,
            "window_height": 1440,
            "headless": True,
            "renderer": "RayTracedLighting",
            "display_options": 3286,  # Set display options to show default grid
        }

    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp(launch_config=CONFIG)
    from omni.isaac.core.utils.extensions import enable_extension
        

    # Default Livestream settings
    simulation_app.set_setting("/app/window/drawMouse", False)
    simulation_app.set_setting("/app/window/drawMouse", False)
    simulation_app.set_setting("/app/livestream/proto", "ws")
    simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 40)
    simulation_app.set_setting("/ngx/enabled", False)
    # set different websocket server port to run RL training on second container
    # simulation_app.set_setting("/app/livestream/websocket/server_port",8886)
    # simulation_app.set_setting("/exts/omni.services.transport.server.http/port",8201)

    # Default URL: http://localhost:8211/streaming/client/ , change 8211 to customised transport server port
    enable_extension("omni.kit.livestream.native")
    enable_extension("omni.isaac.ros_bridge")
    enable_extension("omni.isaac.physics_inspector")
    # enable_extension("omni.isaac.core.articulations.articulation_view")
    return simulation_app


    