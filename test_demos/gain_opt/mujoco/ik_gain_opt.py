import numpy as np


class IKObjective:
    def __init__(self, robot, scene):
        self.robot = robot
        self.robot.start_logging()
        self.scene = scene
        self.goal_x_positions = []
        self.goal_x_positions.append([0.6, -0.25, 0.15])
        self.goal_x_positions.append([0.6, -0.25, 0.15])
        self.goal_x_positions.append([0.6, -0.25, 0.15])

        self.goal_quat = []
        self.goal_quat.append([0.009, 0.72, -0.67, -0.014])
        self.goal_quat.append([0.7071, 0.7071, 0, 0])
        self.goal_quat.append([-0.7071, 0.7071, 0, 0])

    def eval_func(self, gains):
        pgains = gains[:6]
        dgains = gains[6:]

        obj_val = 0
        self.robot.gotoCartPosQuatImpedanceControllerJacTranspose.get_tracking_controller(
            self.robot
        ).pgain_pos = np.array(
            pgains[:3]
        )
        self.robot.gotoCartPosQuatImpedanceControllerJacTranspose.get_tracking_controller(
            self.robot
        ).pgain_quat = np.array(
            pgains[3:]
        )
        self.robot.gotoCartPosQuatImpedanceControllerJacTranspose.get_tracking_controller(
            self.robot
        ).dain = np.array(
            dgains
        )

        for i in range(len(self.goal_x_positions)):
            c_goal_x_pos = self.goal_x_positions[i]
            c_goal_quat = self.goal_quat[i]

            self.scene.reset()

            self.scene.start_logging()
            self.robot.gotoCartPositionAndQuat_ImpedanceCtrlJacTranspose(
                c_goal_x_pos, c_goal_quat, duration=4
            )
            self.scene.stop_logging()
            obj_val += self.calc_error_per_traj(
                des_x_traj=self.robot.des_c_pos, des_quat_traj=self.robot.des_quat
            )
        obj_val = obj_val / len(self.goal_x_positions)
        return obj_val

    def calc_error_per_traj(self, des_x_traj, des_quat_traj):
        robot_x_traj = self.robot.robot_logger.cart_pos
        robot_quat_traj = self.robot.robot_logger.cart_quat

        diff_x_traj = des_x_traj - robot_x_traj
        diff_quat_traj = des_quat_traj - robot_quat_traj

        x_error = np.mean(np.linalg.norm(diff_x_traj, axis=1))
        quat_error = np.mean(np.linalg.norm(diff_quat_traj[:, 1:], axis=1))

        return 10 * (0.5 * x_error + 0.5 * quat_error)


if __name__ == "__main__":
    # from gain_opt.optimizer import CEM
    # from alr_sim.sims.SimFactory import SimRepository
    #
    # sim_factory = SimRepository.get_factory('mujoco')
    # s = sim_factory.create_scene()
    # robot = sim_factory.create_robot(s)
    #
    # s.start()
    # obj = IKObjective(robot, s)
    # init_val = np.array([450.0, 450.0, 450.0, 250.0, 250.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0])
    # optimizer = CEM(init_mean=init_val, init_cov=np.eye(init_val.shape[0])*20, eval_func=obj.eval_func, n_samples=300,
    #                 n_samples_for_update=120)
    # res = optimizer.iter(n_it=20)

    import cma
    from alr_sim.sims.SimFactory import SimRepository
    from alr_sim.core import Scene

    sim_factory = SimRepository.get_factory("mujoco")
    s = sim_factory.create_scene(render=Scene.RenderMode.BLIND)
    robot = sim_factory.create_robot(s)

    s.start()
    obj = IKObjective(robot, s)
    init_val = np.array(
        [450.0, 450.0, 450.0, 250.0, 250.0, 250.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0]
    )
    x, es = cma.fmin2(obj.eval_func, init_val, sigma0=20)
    es.result_pretty()

    # import cma
    # fun = cma.ff.elli
    # x0 = 4 * [2]
    # sigma0 = 1
    # x, es = cma.fmin2(fun, x0, sigma0)
