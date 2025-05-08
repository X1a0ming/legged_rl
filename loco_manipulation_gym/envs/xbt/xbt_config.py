# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin
import sys
from loco_manipulation_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class XbTCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 2048
        num_actions = 8 # 8D
        num_observations = 45 + 187
    class commands( LeggedRobotCfg ):
        curriculum = True
        max_curriculum = 1.5
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-1.5, 1.5] # min max [m/s]
            lin_vel_y = [-1.0, 1.0]   # min max [m/s]
            ang_vel_yaw = [-1, 1]    # min max [rad/s]
            heading = [-3.14, 3.14]
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = True
        static_friction = 1.0
        dynamic_friction = 1.0
        restitution = 0.
        # rough terrain only:
        measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 5 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows= 10 # number of terrain rows (levels)
        num_cols = 20 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.1, 0.1, 0.35, 0.25, 0.2]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces
   
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.3] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'left_leg': 0.0,   # [rad]
            'right_leg': 0.0,   # [rad]

            'left_thigh': 0.0,     # [rad]
            'right_thigh': 0.0,   # [rad]

            'left_calf': 0.0,   # [rad]
            'right_calf': 0.0,    # [rad]

            'left_foot': 0.0,   # [rad]
            'right_foot': 0.0,    # [rad]
        }
        init_joint_angles = { # = target angles [rad] when action = 0.0
            'left_leg': 0.0,   # [rad]
            'right_leg': 0.0,   # [rad]

            'left_thigh': 0.0,     # [rad]
            'right_thigh': 0.0,   # [rad]

            'left_calf': 0.0,   # [rad]
            'right_calf': 0.0,    # [rad]

            'left_foot': 0.0,   # [rad]
            'right_foot': 0.0,    # [rad]
        }
    class rewards:
        class scales:
            termination = -0.8
            tracking_lin_vel = 3.0
            tracking_ang_vel = 1.5
            lin_vel_z = -0.1
            ang_vel_xy = -0.05
            orientation = -2
            torques = -0.0001
            dof_vel = -1e-7
            dof_acc = -1e-7
            base_height = -0.5
            feet_air_time =  0
            collision = -0.1
            feet_stumble = -0.1
            action_rate = -0.0002
            stand_still = -0.01
            dof_pos_limits = -0.9
            arm_pos = -0.
            hip_action_l2 = -0.1



        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.4 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 0.9 # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 1.
        base_height_target = 0.35 # [m]
        max_contact_force = 100. # forces above this value are penalized

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'leg': 50.,'thigh': 50.,'calf': 50.,"foot":20}  # [N*m/rad]
        damping = {'leg': 2,'thigh': 2,'calf': 2,"foot":0.5}      # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class domain_rand( LeggedRobotCfg.domain_rand ):
        randomize_friction = False
        friction_range = [0.1, 2.0]
        randomize_base_mass = False
        added_mass_range = [-0.4, 0.4]
        push_robots = False
        push_interval_s = 3
        max_push_vel_xy = 0.2
        max_push_ang_vel = 0.2

        randomize_base_com = False
        added_com_range = [-0.15, 0.15]
        randomize_motor = False
        motor_strength_range = [0.8, 1.2]

        action_delay = 0.1
        action_noise = 0.01

    class asset( LeggedRobotCfg.asset ):
        file = '{LOCO_MANI_GYM_ROOT_DIR}/resources/robots/XbT/urdf/XbT.urdf'
        name = "XbT"
        arm_name = ""
        foot_name = "foot"
        wheel_name =["foot"]
        penalize_contacts_on = ["thigh", "calf","base"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter "base","calf","hip","thigh"
        replace_cylinder_with_capsule = False
        flip_visual_attachments = True


class XbTCfgPPO( LeggedRobotCfgPPO ):
    seed = 5

    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.003
        learning_rate = 1e-4

    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'XbT'
        resume = False
        num_steps_per_env = 48 # per iteration
        max_iterations = 3000
        # load_run = '/path/to/pretrained_model'
        load_run = None
        checkpoint = 0

  