# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .humanoid_control import (
    HumanoidJointController,
    HumanoidControlExtension,
    get_humanoid_control_extension,
    setup_humanoid_robot,
    get_humanoid_controller
)

def init():
    """
    Initialize the humanoid control extension
    """
    print("[omni.isaac.humanoid_control] Extension initialized")
    return get_humanoid_control_extension()


def shutdown():
    """
    Shutdown the humanoid control extension
    """
    extension = get_humanoid_control_extension()
    if extension:
        extension.cleanup()
    print("[omni.isaac.humanoid_control] Extension shutdown")