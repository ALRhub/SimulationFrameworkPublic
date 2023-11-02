from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box, Sphere
from alr_sim.utils.point_clouds import PcVisualizer

if __name__ == "__main__":
    box = Box(
        name="box",
        init_pos=[0.6, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
        static=False,
    )
    sphere = Sphere(
        name="sphere",
        init_pos=[0.6, 0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[0, 1, 0, 1],
        static=False,
    )
    object_list = [box, sphere]
    duration = 4

    sim_factory = SimRepository.get_factory("pybullet")

    # TODO: unify camelCase / snake_case naming convention
    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)

    print(scene.setup_done)
    scene.add_object(sim_factory.create_camera("test_cam"))

    scene.start()

    home_position = robot.current_c_pos
    scene.start_logging()  # log the data for plotting

    robot.set_gripper_width = 0.04

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.20, 0.02],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    pcv = PcVisualizer()
    xyz, rgb = scene.get_object(name="rgbd_cage").calc_point_cloud()
    pcv.visualize_point_clouds(xyz, rgb)

    print(scene.get_obj_quat(box))
    print(scene.get_obj_pos(obj_name="box"))
    print(scene.get_obj_quat(obj_id=sphere.obj_id))

    scene.stop_logging()
    robot.robot_logger.plot(plot_selection=RobotPlotFlags.END_EFFECTOR)
