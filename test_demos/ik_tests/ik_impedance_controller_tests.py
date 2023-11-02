from test_demos.ik_tests import ik_test_base

if __name__ == "__main__":
    _, rb = ik_test_base.create_test(1.0)

    poses = None

    rb.move_func = rb.gotoCartPositionAndQuat_ImpedanceCtrl
    # poses = np.load("poselog.npy", allow_pickle=True)
    ik_test_base.execute_test(rb, poses)
