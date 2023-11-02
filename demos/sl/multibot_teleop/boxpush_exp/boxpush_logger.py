import glob
import os

import numpy as np

from alr_sim.utils.geometric_transformation import quat2euler


class VTwinLogger:
    """Small Log Writer Class to avoid overriding existing logs"""

    def __init__(self, root_dir="./", c_mng=None) -> None:
        os.makedirs(root_dir, exist_ok=True)
        self.root_dir = root_dir
        self.mng = c_mng

        self.log_counter = len(glob.glob(os.path.join(self.root_dir, "primary_*")))
        self.is_logging = False

    def start_log(self, prim_rb, repl_rb):
        if self.is_logging:
            print("Log already started.")

        self.is_logging = True
        prim_rb.scene.start_logging()
        repl_rb.scene.start_logging()
        print("Start Logfiles {:03d}".format(self.log_counter))

    def abort_log(self, prim_rb, repl_rb):
        if not self.is_logging:
            print("No log in process.")
            return

        self.is_logging = False
        prim_rb.scene.stop_logging()
        repl_rb.scene.stop_logging()

        print("Abort Logfiles {:03d} without save".format(self.log_counter))

    def stop_log(self, prim_rb, repl_rb):
        if not self.is_logging:
            print("No log in process.")
            return

        self.is_logging = False
        prim_rb.scene.stop_logging()
        repl_rb.scene.stop_logging()

        np.savez(
            os.path.join(
                self.root_dir,
                "primary_{:02d}_{:03d}.npz".format(self.mng.index, self.log_counter),
            ),
            time_stamp=prim_rb.robot_logger.time_stamp,
            j_pos=prim_rb.robot_logger.joint_pos,
            c_pos=prim_rb.robot_logger.cart_pos,
        )

        np.savez(
            os.path.join(
                self.root_dir,
                "replica_{:02d}_{:03d}.npz".format(self.mng.index, self.log_counter),
            ),
            time_stamp=repl_rb.robot_logger.time_stamp,
            j_pos=repl_rb.robot_logger.joint_pos,
            c_pos=repl_rb.robot_logger.cart_pos,
        )

        euler = quat2euler(repl_rb.scene.get_obj_quat(obj_name="push_box"))
        box_pos = repl_rb.scene.get_obj_pos(obj_name="push_box")

        target_euler = quat2euler(
            repl_rb.scene.get_obj_quat(obj_name="target_indicator")
        )
        target_pos = repl_rb.scene.get_obj_pos(obj_name="target_indicator")

        with open(os.path.join(self.root_dir, "contexts.txt"), "a") as f:
            f.write(
                "{} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n".format(
                    self.log_counter,
                    target_pos[0],
                    target_pos[1],
                    target_euler[2],
                    box_pos[0],
                    box_pos[1],
                    euler[2],
                )
            )

        print("Saved Logfiles {:03d}".format(self.log_counter))

        self.log_counter += 1
