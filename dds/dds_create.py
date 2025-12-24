# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from dds.dds_master import dds_manager


def create_dds_objects(args_cli, env):
    """Create and register DDS objects for the current simulation."""
    publish_names = []
    subscribe_names = []

    # Robot-specific DDS nodes
    if args_cli.robot_type == "bessica_d":
        from dds.bessica_d_dds import BessicaRobotDDS
        bessica_robot = BessicaRobotDDS()
        dds_manager.register_object("bessica_d", bessica_robot)
        publish_names.append("bessica_d")
        subscribe_names.append("bessica_d")

    # Reset-pose command DDS (subscriber only)
    from dds.reset_pose_dds import ResetPoseCmdDDS
    reset_pose_dds = ResetPoseCmdDDS()
    dds_manager.register_object("reset_pose", reset_pose_dds)
    subscribe_names.append("reset_pose")

    # Simulation state DDS (publisher + subscriber)
    from dds.sim_state_dds import SimStateDDS
    sim_state_dds = SimStateDDS(env, args_cli.task)
    dds_manager.register_object("sim_state", sim_state_dds)
    publish_names.append("sim_state")

    # Start DDS communication
    dds_manager.start_publishing(publish_names)
    dds_manager.start_subscribing(subscribe_names)
    return reset_pose_dds, sim_state_dds, dds_manager


def create_dds_objects_replay(args_cli, env):
    """Create DDS objects for replay-only flows (no sim_state/reset_pose)."""
    publish_names = []
    subscribe_names = []

    if args_cli.robot_type == "bessica_d":
        from dds.bessica_d_dds import BessicaRobotDDS
        bessica_robot = BessicaRobotDDS()
        dds_manager.register_object("bessica_d", bessica_robot)
        publish_names.append("bessica_d")
        subscribe_names.append("bessica_d")

    dds_manager.start_publishing(publish_names)
    dds_manager.start_subscribing(subscribe_names)