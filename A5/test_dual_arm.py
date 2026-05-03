from bimanual import SingleArm
from typing import Dict, Any
import numpy as np

def test_dual_arm(single_arm0: SingleArm, single_arm1: SingleArm):
    while(1):

        single_arm0.gravity_compensation()
        single_arm1.gravity_compensation()

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

    single_arm0 = SingleArm(arm_config_0)
    single_arm1 = SingleArm(arm_config_1)
    test_dual_arm(single_arm0,single_arm1)