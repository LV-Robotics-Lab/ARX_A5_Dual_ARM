from bimanual import SingleArm
from typing import Dict, Any
import numpy as np

def test_dual_arm_with_head(single_arm0: SingleArm, single_arm1: SingleArm, single_arm_head: SingleArm):
    while(1):
        single_arm0.gravity_compensation()
        single_arm1.gravity_compensation()
        single_arm_head.gravity_compensation()



if __name__ == "__main__":
    arm_config_0: Dict[str, Any] = {
        "can_port": "can1",
        "urdf_name": "a5.urdf",
        # Add necessary configuration parameters for the left arm
    }

    arm_config_1: Dict[str, Any] = {
        "can_port": "can3",
        "urdf_name": "a5.urdf",
        # Add necessary configuration parameters for the right arm
    }

    arm_config_2: Dict[str, Any] = {
        "can_port": "can0",
        "urdf_name": "a5_head.urdf",
        # Add necessary configuration parameters for the head
    }

    single_arm0 = SingleArm(arm_config_0)
    single_arm1 = SingleArm(arm_config_1)
    single_arm_head = SingleArm(arm_config_2)
    test_dual_arm_with_head(single_arm0, single_arm1, single_arm_head)