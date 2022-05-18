#!/usr/bin/python3
import os
import time
from threading import Thread,Event
from ruamel.yaml import YAML
from stable_baselines3 import PPO
import numpy as np
from utils import AgileCommandMode, AgileCommand




############### PARAMS ########################
LOAD_PATH = os.path.dirname(os.path.abspath(__file__)) + "/trained-models/out-run-YYYY-MM-DD-HH-MM-SS/best_model/best_model"
USE_DEPTH = False

VISION_EVAL = 1

N_FRAMES = 1
IMAGE_WIDTH = 120
IMAGE_HEIGHT = 120

IMAGE_CHANNELS = 1 if USE_DEPTH else 3

###################### LOAD_YAML ###################
env_cfg = YAML().load(
    open(
        os.environ["FLIGHTMARE_PATH"] + "/flightpy/configs/vision/config.yaml", "r"
    )
)
###################### ACTION_NORMALIZATION_PARAMS ###################
quad_mass = env_cfg["quadrotor_dynamics"]["mass"]
omega_max = env_cfg["quadrotor_dynamics"]["omega_max"]
thrust_max = 4 * env_cfg["quadrotor_dynamics"]["thrust_map"][0] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"]
act_mean = np.array([thrust_max / quad_mass / 2, 0.0, 0.0, 0.0])[np.newaxis, :]
act_std = np.array([thrust_max / quad_mass / 2, \
                      omega_max[0], omega_max[1], omega_max[2]])[np.newaxis, :]

###################### MODEL_VARIABLE ###################
dict_state_key = "drone_state"
dict_image_key = "depth" if USE_DEPTH else "rgb"
dict_obs_key = "obstacles"

###################### STACKED_VARIABLE ###################
stacked_imgs = []


###################### state preparation ###################

def prepare_drone_state(state):
    new_state = []
    state_elem = [state.pos, state.att, state.vel, state.omega]
    for el in state_elem:
        new_state.extend(el)
    return new_state

def stack_frames(frame_list, new_frame):
    if len(frame_list) == 0:
        frame_list = [new_frame for _ in range(N_FRAMES)]
    else:
        frame_list = frame_list[:N_FRAMES - 1]
        frame_list.insert(0, new_frame)
    return frame_list


def _normalize(value, min, max):
    return 2 * (value - min) / (max - min) - 1

def normalize_drone_state(drone_state):
    drone_state[:, 0] = _normalize(drone_state[:, 0], -10, 65)
    drone_state[:, 1] = _normalize(drone_state[:, 1], -10, 10)
    drone_state[:, 2] = _normalize(drone_state[:, 2], 0, 10)
    drone_state[:, 7] = _normalize(drone_state[:, 7], -3, 65)
    drone_state[:, 8] = _normalize(drone_state[:, 8], -25, 30)
    drone_state[:, 9] = _normalize(drone_state[:, 9], -20, 20)
    drone_state[:, 10] = _normalize(drone_state[:, 10], -9, 9)
    drone_state[:, 11] = _normalize(drone_state[:, 11], -9, 9)
    drone_state[:, 12] = _normalize(drone_state[:, 12], -9, 9)

    return drone_state.copy()


def normalize_state_obstacles(obstacles):
    obstacles[:, 0::4] = _normalize(obstacles[:, 0::4], -8, 1010)
    obstacles[:, 1::4] = _normalize(obstacles[:, 1::4], -20, 1008)
    obstacles[:, 2::4] = _normalize(obstacles[:, 2::4], -10, 1000)
    obstacles[:, 3::4] = _normalize(obstacles[:, 3::4], 0, 1.5)

    return obstacles.copy()





#################### load ppo model ########################

model_ppo = PPO.load(LOAD_PATH, env=None, device="cuda:0")
print("LOADED!")


################## ping thread ##########################

last_state_vision = {
        dict_state_key: np.zeros((1,13)),
        dict_image_key: np.zeros((1,IMAGE_CHANNELS,IMAGE_HEIGHT,IMAGE_WIDTH))
        }

last_state_st_based = {
        dict_state_key: np.zeros((1,13)),
        dict_obs_key: np.zeros((1,40))
        }

last_action_vision = np.array([[0, 0, 0, 0]])
last_action_st_based = np.array([[0, 0, 0, 0]])

stopFlag = Event()

class PingThreadVision(Thread):
    def __init__(self,event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        global last_action_vision
        while True:
            while not self.stopped.wait(0.00001):
                last_action_vision, _ = model_ppo.predict(last_state_vision, deterministic=True)


class PingThreadStateBased(Thread):
    def __init__(self,event):
        Thread.__init__(self)
        self.stopped = event

    def run(self):
        global last_action_st_based
        while True:
            while not self.stopped.wait(0.00001):
                last_action_st_based, _ = model_ppo.predict(last_state_st_based, deterministic=True)

if(VISION_EVAL != 0):
	thread = PingThreadVision(stopFlag)
	model_ppo.predict(last_state_vision, deterministic=True)
else:
	thread = PingThreadStateBased(stopFlag)
	model_ppo.predict(last_state_st_based, deterministic=True)
thread.daemon = True
thread.start()


################### compute action #######################

def compute_command_vision_based(state, img):
    #    global stacked_drone_state
    global stacked_imgs
    global last_state_vision
    ############ LOAD_STATE #########

    drone_state = np.array(prepare_drone_state(state))
    drone_state = np.expand_dims(drone_state, 0)
    drone_state = normalize_drone_state(drone_state)

    ############ LOAD_IMAGE #########

    stacked_imgs = stack_frames(stacked_imgs, img)
    new_img = np.array(stacked_imgs)
    new_img = np.expand_dims(new_img, 0)

    obs = {
        dict_state_key: drone_state,
        dict_image_key: new_img
    }

    last_state_vision = obs

    action = last_action_vision
    ############ NORMALIZE_ACTION #########
    #action = np.array([[-0.68, 0, 0.5, 0]])
    action = (action * act_std + act_mean)[0, :]


    # CTBR command
    command_mode = AgileCommandMode.CTBR
    command = AgileCommand(command_mode)
    command.t = state.t
    command.collective_thrust = action[0]
    command.bodyrates = [action[1], action[2], action[3]]
    return command


def compute_command_state_based(state, obstacles, rl_policy=None):
    global last_state_st_based

    drone_state = np.array(prepare_drone_state(state))
    drone_state = np.expand_dims(drone_state, 0)
    drone_state = normalize_drone_state(drone_state)
    obstacle_state = []
    for x in obstacles.obstacles:
        partial_vec1 = [x.position.x, x.position.y, x.position.z, x.scale]
        obstacle_state.append(partial_vec1)
    obstacle_state = np.array(obstacle_state)
    obstacle_state = np.expand_dims(obstacle_state, 0)
    obstacle_state = normalize_state_obstacles(obstacle_state)
    obs = {
        dict_state_key: drone_state,
        dict_obs_key: np.array(obstacle_state)
    }

    last_state_st_based = obs

    action = last_action_st_based
    ############ NORMALIZE_ACTION #########
    action = (action * act_std + act_mean)[0, :]

    # CTBR command
    command_mode = 1
    command = AgileCommand(command_mode)
    command.t = state.t
    command.collective_thrust = action[0]
    command.bodyrates = [action[1], action[2], action[3]]
    return command
