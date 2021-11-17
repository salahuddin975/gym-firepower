import gym
from gym import error, spaces, utils
from gym.spaces.space import Space
from gym.utils import seeding
from pypower.idx_brch import F_BUS, T_BUS
from fire_spread import FireSpread
from power_sim import PowerOperations
from pypower.loadcase import loadcase
import numpy as np
from math import ceil
from pypower.idx_bus import *
from pypower.idx_gen import *
from pypower.idx_brch import *
from pypower.ext2int import ext2int
from copy import deepcopy


class FirePowerEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, geo_file=None, network_file=None, scaling_factor=None,
                non_convergence_penalty=None, protection_action_penalty=None,
                active_line_removal_penalty=None, sampling_duration=1/6, num_tunable_gen=10, seed=None):
        if geo_file is None:
            assert False, 'Configuration file for fire spreading model is missing'
        if network_file is None:
            assert False, 'Power system network file is missing'

        self.seed(seed)
        self.geo_file = geo_file
        self.network_file = network_file
        self.scaling_factor = scaling_factor
        self.sampling_duration = sampling_duration
        self.num_tunable_gen = num_tunable_gen

        ppc = loadcase(network_file)
        PowerOperations.mergeGenerators(ppc)
        PowerOperations.mergeBranches(ppc)
        self.ppc = ext2int(ppc)
        assert num_tunable_gen <= self.ppc["gen"].shape[0], "Number of tunable generators should be less than total generators"

        self.fire_spread_model = FireSpread(geo_file, scaling_factor, self.np_random)
        self.power_sys_model = PowerOperations(self.ppc, self.fire_spread_model.get_reduced_state(),
                                    sampling_duration, num_tunable_gen)

        self.viewer = None
        self._set_penalties(non_convergence_penalty, protection_action_penalty, active_line_removal_penalty)
        self.observation_space = self._create_observation_space()
        self.action_space = self._create_action_space()

    def _set_penalties(self, non_convergence_penalty, protection_action_penalty, active_line_removal_penalty):
        self.total_load = np.sum(ceil(sum(self.ppc['bus'][:, PD])))

        if non_convergence_penalty is None:
            elements = self.ppc['bus'].shape[0] + self.ppc['branch'].shape[0]
            self.nc_penalty = -10 * elements * self.total_load
        else:
            self.nc_penalty = non_convergence_penalty

        if protection_action_penalty is None:
            self.pa_penalty = -2 * self.total_load
        else:
            self.pa_penalty = protection_action_penalty

        if active_line_removal_penalty is None:
            self.la_penalty = -1 * self.total_load
        else:
            self.la_penalty = active_line_removal_penalty

        assert self.pa_penalty < 0 and self.nc_penalty < 0 and self.la_penalty < 0, "Penalties should be negative"
        assert self.la_penalty > self.pa_penalty > self.nc_penalty, "Non convergence must be penalized more \
        strongly than automatic protection action followed by active line removal action"

    def step(self, action):
        self._take_action(action)
        status = self._get_status()
        reward = self._get_reward()
        observation = self._get_state()
        return observation, reward, status, {}
    
    def reset(self):
        self.fire_spread_model.reset()
        self.power_sys_model.reset()
        return self._get_state()
    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode='human'):
        img = self.fire_spread_model.get_current_image()
        if mode == 'rgb_array':
            return img
        elif mode == 'human':
            from gym.envs.classic_control import rendering
            if self.viewer is None:
                self.viewer = rendering.SimpleImageViewer()
            self.viewer.imshow(img)
            return self.viewer.isopen

    def close(self):
        pass

    def _take_action(self, action):
        self.fire_spread_model.step()
        fire_state = self.fire_spread_model.get_reduced_state()
        self.power_sys_model.step(action, fire_state)

    def _get_reward(self):
        if self.power_sys_model.has_converged:
            load_loss = -1 * (self.ppc["baseMVA"] * self.power_sys_model.get_load_loss())
            protection_penalty = self.power_sys_model.get_protection_operation_count() * self.pa_penalty
            active_line_removal = -1 * self.power_sys_model.get_active_line_removal_penalty()

            return load_loss + protection_penalty + active_line_removal, load_loss
        else:
            return (self.nc_penalty, 0)

    def _get_state(self):
        power_state = self.power_sys_model.get_state()
        fire_state = self.fire_spread_model.get_state()
        power_state.update({'fire_state': fire_state})
        fire_distance = self.fire_spread_model.get_distance_from_fire()
        # print(fire_distance)
        fire_distance_list = []
        for bus in self.ppc["bus"][:, BUS_I]:
            fire_distance_list.append(fire_distance["nodes"][int(bus)])
        for branch in self.ppc["branch"]:
            from_bus = int(branch[F_BUS])
            to_bus = int(branch[T_BUS])
            fire_distance_list.append(fire_distance["branches"][(from_bus, to_bus)])
        power_state.update({'fire_distance': fire_distance_list})
        return power_state
    
    def _get_status(self):
        return self.power_sys_model.get_status()

    def _create_action_space(self):
        num_branch = self.ppc["branch"].shape[0]
        branch_space = spaces.MultiBinary(num_branch)
        num_bus = self.ppc["bus"].shape[0]
        bus_space = spaces.MultiBinary(num_bus)
        # lower_bound = np.zeros(num_bus, np.float64)
        max_ramp = max(self.ppc["gen"][:, RAMP_10])/self.ppc["baseMVA"]
        lower_bound = -1*self.sampling_duration*max_ramp*np.ones(
            self.num_tunable_gen, np.float32)
        upper_bound = self.sampling_duration*max_ramp*np.ones(
            self.num_tunable_gen, np.float32)
        # upper_bound = np.zeros(num_bus, np.float64)
        # for gen in self.ppc["gen"]:
        #     lower_bound[int(gen[GEN_BUS])] = -1*self.sampling_duration*gen[RAMP_10]/self.ppc["baseMVA"]
        #     upper_bound[int(gen[GEN_BUS])] = self.sampling_duration*gen[RAMP_10]/self.ppc["baseMVA"]
        gen_space = spaces.Box(
            low=lower_bound, high=upper_bound, shape=(self.num_tunable_gen,))
        
        gen_selector_space = spaces.MultiDiscrete([self.ppc["bus"].shape[0]+1]*self.num_tunable_gen)
        
        return spaces.Dict({"generator_injection": gen_space,
                            "branch_status": branch_space,
                            "bus_status": bus_space,
                            "generator_selector":gen_selector_space})
        
    def _create_observation_space(self):
        num_branch = self.ppc["branch"].shape[0]
        branch_space = spaces.MultiBinary(num_branch)
        num_bus = self.ppc["bus"].shape[0]
        bus_space = spaces.MultiBinary(num_bus)

        # This is 0 instead of PMIN as generator can be disconnected
        gen_lower_bound = np.zeros((num_bus,), np.float32) 
        gen_upper_bound = np.zeros((num_bus,), np.float32)
        for gen in self.ppc["gen"]:
            gen_upper_bound[int(gen[GEN_BUS])] = gen[PMAX]/self.ppc["baseMVA"]
        gen_space = spaces.Box(low=gen_lower_bound, high=gen_upper_bound, shape=(num_bus, ))

        load_lower_bound = np.zeros((num_bus,), np.float32)
        load_upper_bound = np.array(self.ppc["bus"][:, PD]/self.ppc["baseMVA"])
        load_space = spaces.Box(low=load_lower_bound, high=load_upper_bound, shape=(num_bus, ))
        
        theta_lower_bound = -1*(np.pi/4)*np.ones((num_bus,), np.float32)
        theta_upper_bound = (np.pi/4)*np.ones((num_bus,), np.float32)
        theta_space = spaces.Box(low=theta_lower_bound, high=theta_upper_bound, shape=(num_bus, ))
        
        # 0 -> unbunt, 1 -> burning, 2 -> burnt
        fire_space = spaces.Box(low=0, high=2, shape=(self.fire_spread_model.grid.rows,
                                self.fire_spread_model.grid.cols), dtype=np.uint8)
        
        fire_distance_space = spaces.Box(low=0, high=np.sqrt(self.fire_spread_model.grid.rows**2 + \
                                    self.fire_spread_model.grid.cols**2), 
                                    shape=(num_bus+num_branch, ), dtype=np.float32)

        line_flow_upper = self.ppc["branch"][:, RATE_A] / self.ppc["baseMVA"]
        line_flow_lower = -1*self.ppc["branch"][:, RATE_A] / self.ppc["baseMVA"]
        line_flow_space = spaces.Box(low=line_flow_lower, high=line_flow_upper, shape=(num_branch, ), dtype=np.float32)

        return spaces.Dict({"generator_injection": gen_space,
                            "load_demand":  load_space,
                            "theta": theta_space,
                            "branch_status": branch_space,
                            "bus_status": bus_space,
                            "fire_status": fire_space,
                            "fire_distance": fire_distance_space,
                            "line_flow": line_flow_space})
