from pypower.loadcase import loadcase
from pypower.idx_gen import *
from pypower.idx_brch import *
from pypower.idx_bus import *
from pypower.int2ext import *
from pypower.ext2int import *
import numpy as np
from gym_firepower.envs.gams.models import initial_model, run_time_model
from gams import *
import sys
import os
import pathlib
from pypower.rundcpf import *
from pprint import PrettyPrinter
from gym import logger
import glob

pp = PrettyPrinter(compact=True, depth=3)

class PowerOperations(object):
    def __init__(self, ppc_int, initial_fire_state, sampling_duration, num_tunable_generator):
        assert sampling_duration > 0, "Sampling duration should be a positive number"
        self.sampling_duration = sampling_duration

        self.ppc_int = ppc_int                 # This ppc is based on the internal numbering
        self.ppc_ext = int2ext(ppc_int)        # This ppc is based on external numbering

        # Initial Load Flow based on internal numbering
        self.initial_load_flow = rundcpf(self.ppc_ext)
        if self.initial_load_flow[1]:
            self.initial_load_flow = ext2int(self.initial_load_flow[0])
        else:
            raise Exception("Unsuccessful load flow calculation")

        self.num_tunable_generator = num_tunable_generator
        self.num_bus = self.ppc_int["bus"].shape[0]                   # Number of buses
        self.num_branch = self.ppc_int["branch"].shape[0]             # Number of branches

        self.from_buses = self.ppc_int["branch"][:, F_BUS].astype('int')           # List of 'from' buses
        self.to_buses = self.ppc_int["branch"][:, T_BUS].astype('int')             # List of 'to' buses

        self.p_load_upper = self.ppc_int["bus"][:, PD] / self.ppc_int["baseMVA"]   # List of upper threshold of load demand on each bus
        self.non_crtitcal_fractional = self.ppc_int["noncrticalfrac"]    # Portion of a load at bus thats critical
        self.weights = self.ppc_int["weights"]                 # Weight associated with critical, non-critical load

        gen_buses = self.ppc_int["gen"][:,0]
        self.gen_buses = np.vectorize(lambda x: int(x))(gen_buses)

        self._cretae_ybus()        # Creating ybus
        self.B = -1j * self.ybus
        self.B = self.B.astype('float64')       # ommitting the imaginary part and converting to float

        self.initial_model_text = initial_model
        self.run_time_model_text = run_time_model

        self.initial_fire_state = initial_fire_state
        self.initial_action = {"generator_injection": np.zeros(self.num_tunable_generator, dtype=np.float32),
                               "branch_status": np.ones(self.num_branch, dtype=int),
                               "bus_status": np.ones(self.num_bus, dtype=int),
                               "generator_selector": np.array([self.num_bus]*self.num_tunable_generator, dtype=int)}

        path = pathlib.Path(__file__).parent.absolute()               # Identify artifacts location
        self.gams_dir = os.path.join(path, "gams", "temp", "pid_{}".format(os.getpid()) )
        try:
            os.makedirs(self.gams_dir, exist_ok=True)
        except OSError:
            assert False, "Cannot create temporary directory"

        self.ws = GamsWorkspace(working_directory=self.gams_dir, debug=DebugLevel.Off)
        self.db = self.ws.add_database()

        self._define_problem()

    def _initialize(self):
        self.bus_status = np.ones(self.num_bus, dtype=int)             # Status of every bus
        self.branch_status = np.zeros((self.num_bus, self.num_bus), dtype=np.float32)

        self.power_flow_line = np.zeros((self.num_bus, self.num_bus), dtype=np.float32)
        self.power_flow_line_upper = np.zeros((self.num_bus, self.num_bus), dtype=np.float32)

        for branch_ctr in range(self.num_branch):
            r = self.from_buses[branch_ctr]
            c = self.to_buses[branch_ctr]

            # branch -> (r,c)
            self.power_flow_line_upper[r][c] += self.ppc_int["branch"][branch_ctr, RATE_A]/ self.ppc_int["baseMVA"]
            self.branch_status[r][c] = 1
                
            # branch -> (c,r)
            self.power_flow_line_upper[c][r] += self.ppc_int["branch"][branch_ctr, RATE_A]/ self.ppc_int["baseMVA"]
            self.branch_status[c][r] = 1

        self.p_load_initial = self.ppc_int["bus"][:,PD]/ self.ppc_int["baseMVA"]      # List of load demand on each bus
        self.p_load = deepcopy(self.p_load_initial)

        self.pg_upper = np.zeros(self.num_bus, np.float64)           # non generator buses with upper limit set to 0
        self.pg_lower = np.zeros(self.num_bus, np.float64)
        self.max_ramp = np.zeros(self.num_bus, np.float64)
        self.pg_injection = np.zeros(self.num_bus, np.float64)
        
        for gen in self.initial_load_flow["gen"]:
            self.pg_lower[int(gen[GEN_BUS])] = gen[PMIN] / self.ppc_int["baseMVA"]
            self.pg_upper[int(gen[GEN_BUS])] = gen[PMAX] / self.ppc_int["baseMVA"]
            self.max_ramp[int(gen[GEN_BUS])] = gen[RAMP_10] / self.ppc_int["baseMVA"]
            self.pg_injection[int(gen[GEN_BUS])] = gen[PG] / self.ppc_int["baseMVA"]

        sub_numbers = [1.9200, 1.9200, 0.0000, 0.0000,
                    0.0000, 0.0000, 3.0000, 0.0000,
                    0.0000, 0.0000, 0.0000, 0.0000,
                    5.9100, 0.0000, 2.1500, 1.5500,
                    0.0000, 1.9833, 0.0000, 0.0000,
                    1.9833, 3.0000, 5.0833, 0.0000]
        self.pg_injection = sub_numbers              # Overwriting Load Flow calculation with subir's numbers

        self.theta_upper = (np.pi/4)*np.ones(self.num_bus, dtype=np.float64)        # Voltage angle upper limit is pi/4
        self.theta_lower = -1*(np.pi/4)*np.ones(self.num_bus, dtype=np.float64)     # Voltage angle lower limit is -pi/4
        self.theta = np.zeros(self.num_bus, dtype=np.float64)
        
        # Setting and Running the optimizatin problem for the base case
        # This is better than just load flow as load flow will not
        # take care of line limits. The other option could have been 
        # dc opf in pypower
        self._solve_initial_model()
        
        self.previous_action = deepcopy(self.initial_action)
        self.previous_fire_state = deepcopy(self.initial_fire_state)
        self.protection_action_count = 0
        self.live_equipment_removal_count = 0
        self.live_equipment_removal_penalty = 0
        
    def _cretae_ybus(self):
        self.ybus = np.zeros((self.num_bus, self.num_bus), dtype=np.complex_)
        # branch resistance is assumed to be zero (DC loadflow)
        self.branch_impedance = 1j * self.ppc_int["branch"][:, BR_X]
        self.branch_admittance = 1/ self.branch_impedance
        for branch_ctr in range(0,self.num_branch):
            self.ybus[self.from_buses[branch_ctr], self.to_buses[branch_ctr]] -= self.branch_admittance[branch_ctr]
            self.ybus[self.to_buses[branch_ctr], self.from_buses[branch_ctr]] = self.ybus[self.from_buses[branch_ctr], self.to_buses[branch_ctr]]
        
        for bus_ctr in range(0, self.num_bus):
            for branch_ctr in range(0, self.num_branch):
                if self.from_buses[branch_ctr] == bus_ctr or self.to_buses[branch_ctr] == bus_ctr:
                    self.ybus[bus_ctr, bus_ctr] += self.branch_admittance[branch_ctr]
    
    def _setup_problem(self, opt_problem):
        self.db.clear()
        for ctr in range(1, 3):
            self.critical_load.add_record(str(ctr))
        for ctr_bus1 in range(self.num_bus):
            self.nodes.add_record(str(ctr_bus1))
            self.PGLbarT.add_record(str(ctr_bus1)).value = self.pg_lower[ctr_bus1]
            self.PGUbarT.add_record(str(ctr_bus1)).value = self.pg_upper[ctr_bus1]
            self.ThetaLbar.add_record(str(ctr_bus1)).value = self.theta_lower[ctr_bus1]
            self.ThetaUbar.add_record(str(ctr_bus1)).value = self.theta_upper[ctr_bus1]
            self.PLoad.add_record(str(ctr_bus1)).value = self.p_load[ctr_bus1]
            self.Rampbar.add_record(str(ctr_bus1)).value = self.max_ramp[ctr_bus1]
            self.PGBegin.add_record(str(ctr_bus1)).value = self.pg_injection[ctr_bus1]
            for ctr_bus2 in range(self.num_bus):
                self.B_.add_record((str(ctr_bus1), str(ctr_bus2))).value = float(self.B[ctr_bus1][ctr_bus2])
                self.PLbar.add_record((str(ctr_bus1), str(ctr_bus2))).value = float(self.power_flow_line_upper[ctr_bus1][ctr_bus2])
                self.LineStat.add_record((str(ctr_bus1), str(ctr_bus2))).value = float(self.branch_status[ctr_bus1][ctr_bus2])
            
            self.CritFrac.add_record((str(ctr_bus1), "1")).value = float(self.non_crtitcal_fractional[ctr_bus1][1])
            self.CritFrac.add_record((str(ctr_bus1), "2")).value = 1- float(self.non_crtitcal_fractional[ctr_bus1][1])
        
        for ctr_c in range(2):
            self.CritVal.add_record(str(ctr_c+1)).value = float(self.weights[ctr_c])
        self.IntDur.add_record().value = self.sampling_duration
        self.problem = self.ws.add_job_from_string(opt_problem)
        opt = self.ws.add_options()
        opt.defines["gdxincname"] = self.db.name
        self.problem.run(opt, databases=self.db)

    def _define_problem(self):
        self.nodes = self.db.add_set("nodes", 1, "Set of nodes")
        self.critical_load = self.db.add_set("critical_load", 1, "Load divided based on criticality")
        
        self.PGLbarT = self.db.add_parameter_dc("PGLbarT", [self.nodes], "Power generation lower limit")
        self.PGUbarT = self.db.add_parameter_dc("PGUbarT", [self.nodes], "Power generation upper limit")

        self.ThetaLbar = self.db.add_parameter_dc("ThetaLbar", [self.nodes], "Power angle lower limit")
        self.ThetaUbar = self.db.add_parameter_dc("ThetaUbar", [self.nodes], "Power angle upper limit")

        self.B_ = self.db.add_parameter_dc("B", [self.nodes, self.nodes], "Admittance matrix")
        self.PLoad = self.db.add_parameter_dc("PLoad", [self.nodes], "Load value")
        self.PLbar = self.db.add_parameter_dc("PLbar", [self.nodes, self.nodes], "Line Flow Limits")

        self.LineStat = self.db.add_parameter_dc("LineStat", [self.nodes, self.nodes], "Line Status")
        self.Rampbar = self.db.add_parameter_dc("Rampbar", [self.nodes], "Ramp rate limit")
        self.IntDur = self.db.add_parameter("IntDur", 0, "Duration of a typical interval")

        self.CritFrac = self.db.add_parameter_dc("CritFrac", [self.nodes, self.critical_load], "Fraction of loads critical and non-critical")
        self.CritVal = self.db.add_parameter_dc("CritVal", [self.critical_load], "Value of the critical load")
        self.PGBegin = self.db.add_parameter_dc("PGBegin", [self.nodes], "Generation at the begining")

    def _solve_initial_model(self):        
        self._setup_problem(initial_model)
        self._extract_results()
        assert self.has_converged, "Initial Model did not converge"

        self.theta_initial = deepcopy(self.theta)
        self.power_flow_line_initial = deepcopy(self.power_flow_line)
        self.ramp_upper_initial = deepcopy(self.max_ramp)
        self.pg_upper_initial = deepcopy(self.pg_upper)
        self.pg_lower_initial = deepcopy(self.pg_lower)

    def _solve_runtime_model(self):
        self._setup_problem(run_time_model)
        self._extract_results(True)

    def _extract_results(self, induce_violation=False):
        self.p_load_solved = 0.0
        self.p_load_solved_distribution = np.array([0,0], dtype=float)     # [critical load, non critical load]
        model_status  = int(self.problem.out_db["ModStat"].first_record().value)

        if model_status not in [3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 18, 19]:
            self.has_converged = True
            self.theta = [self.problem.out_db["Theta"]["{}".format(i)].get_level() for i in range(self.num_bus)]
            self.pg_injection = np.around([self.problem.out_db["PGn"]["{}".format(i)].get_level() for i in range(self.num_bus)], 4)

            for i in range(self.num_branch):
                from_bus = self.from_buses[i]
                to_bus = self.to_buses[i]
                self.power_flow_line[from_bus][to_bus] = self.problem.out_db["LineFlow"][(str(from_bus), str(to_bus))].get_level()
                self.power_flow_line[to_bus][from_bus] = self.problem.out_db["LineFlow"][(str(to_bus), str(from_bus))].get_level() 

            self.zval = self.problem.out_db["ZVal"].first_record().value
            self.load_loss = self.problem.out_db["Load_loss"].first_record().value
            self.p_load_solved_distribution[0] = np.around(self.problem.out_db["p_solved_c"].first_record().value, 3)
            self.p_load_solved_distribution[1] =  np.around(self.problem.out_db["p_solved_nc"].first_record().value, 3)
            self.p_load_solved =  round(self.problem.out_db["p_solved"].first_record().value, 3)
        else:
            self.has_converged = False
            print("Model did not converge, status {}".format(model_status))

    def get_state(self):
        state = {}
        state["generator_injection"] = self.pg_injection
        state["load_demand"] = self.p_load
        state["branch_status"] = np.array([self.branch_status[self.from_buses[ctr]][self.to_buses[ctr]] for ctr in range(self.num_branch)])
        state["theta"] = self.theta
        state["bus_status"] = self.bus_status
        state["line_flow"] = np.array([self.power_flow_line[self.from_buses[ctr]][self.to_buses[ctr]] for ctr in range(self.num_branch)])
        return deepcopy(state)

    def _check_network_violation(self, action):
        for ctr in range(self.num_bus):
            bus_status = action["bus_status"][ctr]
            for ctr2 in range(self.num_branch):
                if ctr in [self.from_buses[ctr2], self.to_buses[ctr2]]:
                    if bus_status == 0  and action["branch_status"][ctr2] == 1:
                        logger.warn("Network Violation betweem bus {} and  branch ({}, {})".format(ctr, self.from_buses[ctr2], self.to_buses[ctr2]))
                        return True

    def _check_power_generation_injection_cumulative_violations(self, action):
        injections = {int(key): 0 for key in action["generator_selector"]}
        for gen_pair in zip(action["generator_selector"], action["generator_injection"]):
            injections[gen_pair[0]] += round(gen_pair[1],4)

        for gen_bus in injections:
            if gen_bus in self.ppc_int["gen"][:, GEN_BUS] and self.bus_status[gen_bus] != 0:
                if abs(injections[gen_bus]) - self.ramp_upper_initial[gen_bus]  > 0.0001:
                    logger.warn("Cumulative generator ramp rate violation at gen bus {}, Min :{}, Current :{} "
                                "and Max: {}".format(gen_bus, -1*self.ramp_upper_initial[gen_bus], injections[gen_bus],
                                self.ramp_upper_initial[gen_bus]))
                    return True

                if injections[gen_bus] + self.pg_injection[gen_bus] - self.pg_upper_initial[gen_bus] > 0.0001 or \
                    injections[gen_bus] + self.pg_injection[gen_bus] - self.pg_lower_initial[gen_bus] < -0.0001 :
                    logger.warn("Cumulative generator output violation at bus {}".format(gen_bus))
                    logger.debug("Min : {}, Current : {} and Max : {}".format(self.pg_lower_initial[gen_bus],
                        injections[gen_bus] + self.pg_injection[gen_bus], self.pg_upper_initial[gen_bus]))
                    logger.debug("Current output {}".format(self.pg_injection[gen_bus]))
                    logger.debug("Current injection {}".format(injections[gen_bus]))
                    return True

    def _check_generator_ramp_rate_violation(self, action):
        for i in range(self.num_tunable_generator):
            gen_bus = int(action["generator_selector"][i])
            if gen_bus in self.ppc_int["gen"][:, GEN_BUS] and self.bus_status[gen_bus] != 0:
                if not -1*self.ramp_upper_initial[gen_bus] <= action["generator_injection"][i] <= self.ramp_upper_initial[gen_bus]:
                    logger.warn("Gnerator ramp rate violation at gen bus {}, ".format(gen_bus))
                    logger.debug("Min : {}, Current : {} and Max : {}".format(self.pg_lower_initial[gen_bus],
                        action["generator_injection"][i], self.pg_upper_initial[gen_bus]))
                    return True


    def _check_violations(self, action):
        assert action["generator_injection"].shape[0] == self.num_tunable_generator, "generator action dimenion miss-match "
        assert action["branch_status"].shape[0] == self.num_branch, "branch action dimenion miss-match "
        assert action["bus_status"].shape[0] == self.num_bus, "bus action dimenion miss-match "
        assert action["generator_selector"].shape[0] == self.num_tunable_generator, "generator selector dimenion miss-match "
        
        if self._check_network_violation(action):    # node(x) = 0/1 then all branch(x,y) = 0/1
            return True

        if self._check_power_generation_injection_cumulative_violations(action):
            return True

        if self._check_generator_ramp_rate_violation(action):
            return True

        # update pg_upper and pg_lower
        injections = {int(key): 0 for key in action["generator_selector"]}
        for gen_bus in self.ppc_int["gen"][:,GEN_BUS]:
            gen_bus = int(gen_bus)
            if action["bus_status"][gen_bus] == 0 or self.bus_status[gen_bus] == 0:
                continue

            if gen_bus in action["generator_selector"]:
                # fixing PG_injections to the value provided by the agent
                self.pg_injection[gen_bus] = self.pg_injection[gen_bus] + injections[gen_bus]
                self.pg_lower[gen_bus] = self.pg_injection[gen_bus]
                self.pg_upper[gen_bus] = self.pg_injection[gen_bus]
            else:
                # for free generators to initial values(P_MAX, P_MIN)
                self.pg_lower[gen_bus] = self.pg_lower_initial[gen_bus]
                self.pg_upper[gen_bus] = self.pg_upper_initial[gen_bus]
        
        return False
    
    def _check_protection_system_actions(self, fire_state):
        protection_action_count = 0

        for branch in fire_state["branch"]:
            if fire_state["branch"][branch] != self.previous_fire_state["branch"][branch]:
                if fire_state["branch"][branch] == 0:
                    if self.branch_status[branch[0]][branch[1]] == 1:
                        protection_action_count += 1
                        logger.info("Protection action at line ({}, {})".format(branch[0], branch[1]))

                    self.branch_status[branch[0]][branch[1]] = 0
                    self.branch_status[branch[1]][branch[0]] = 0
                    self.power_flow_line_upper[branch[0]][branch[1]] = 0
                    self.power_flow_line_upper[branch[1]][branch[0]] = 0
        
        for node in fire_state["node"]:
            if fire_state["node"][node] != self.previous_fire_state["node"][node]:
                if fire_state["node"][node] == 0:
                    if self.pg_injection[node] > 0.001:
                        protection_action_count += 1
                        logger.info("Protection action at bus {}".format(node))

                    self.pg_injection[node] = 0
                    self.pg_lower[node] = 0
                    self.pg_upper[node] = 0
                    self.max_ramp[node] = 0
                    self.p_load[node] = 0
                    self.p_load_upper[node] = 0
                    self.bus_status[node] = 0 

        self.protection_action_count += protection_action_count

    def _check_live_line_removal_actions(self, branch_actions, node_actions):
        live_equipment_removal_count = 0
        for ctr in range(self.num_branch):
            assert self.branch_status[self.from_buses[ctr]][self.to_buses[ctr]] == \
                self.branch_status[self.to_buses[ctr]][self.from_buses[ctr]], "Branch Status symmetry has been lost"

            if self.branch_status[self.from_buses[ctr]][self.to_buses[ctr]] != branch_actions[ctr]:
                f_bus = self.from_buses[ctr]
                t_bus = self.to_buses[ctr]

                if branch_actions[ctr] == 0:
                    if self.power_flow_line[f_bus][t_bus] > 0.001 or self.power_flow_line[t_bus][f_bus] > 0.001 :
                        logger.warn("Removing a live line ({}, {}) amount of power: {}".format(f_bus, t_bus, self.power_flow_line[f_bus][t_bus]))
                        live_equipment_removal_count += 1
                        self.live_equipment_removal_penalty += abs(self.power_flow_line[f_bus][t_bus])*self.ppc_int["baseMVA"]

                    self.branch_status[f_bus][t_bus] = 0
                    self.branch_status[t_bus][f_bus] = 0
                    self.power_flow_line_upper[f_bus][t_bus] = 0
                    self.power_flow_line_upper[t_bus][f_bus] = 0

        for ctr in range(self.num_bus):
            if self.bus_status[ctr] != node_actions[ctr]:
                if node_actions[ctr] == 0:
                    if self.pg_injection[ctr] > 0.001:
                        logger.warn("Removing a live generator at bus {}, pg_injection is {}".format(ctr, self.pg_injection[ctr]))
                        live_equipment_removal_count += 1
                        gen_bus = int(np.where(self.gen_buses == ctr)[0][0])
                        self.live_equipment_removal_penalty += abs((self.pg_injection[ctr] * self.ppc_int["baseMVA"]) - self.ppc_int["gen"][gen_bus, PMIN])

                    self.pg_injection[ctr] = 0
                    self.pg_lower[ctr] = 0
                    self.pg_upper[ctr] = 0
                    self.max_ramp[ctr] = 0
                    self.p_load[ctr] = 0
                    self.p_load_upper[ctr] = 0
                    self.bus_status[ctr] = 0

        self.live_equipment_removal_count += live_equipment_removal_count

    def _any_change_action(self, action):
        injections = {int(key): 0 for key in action["generator_selector"]}
        for gen_pair in zip(action["generator_selector"], action["generator_injection"]):
            injections[gen_pair[0]] += gen_pair[1]

        previous_injections = {int(key): 0 for key in self.previous_action["generator_selector"]}
        for gen_pair in zip(self.previous_action["generator_selector"], self.previous_action["generator_injection"]):
            previous_injections[gen_pair[0]] += gen_pair[1]

        if not injections == previous_injections:
            logger.info("Injections in the current action are different")
            return True

        if not np.all(action["branch_status"] == self.previous_action["branch_status"]):
            logger.info("Branch status in the current action is different")
            return True

        if not np.all(action["bus_status"] == self.previous_action["bus_status"]):
            logger.info("Bus status in the current action is different")
            return True

        for gen in injections:
            if gen in self.ppc_int["gen"][:, GEN_BUS]:
                if injections[gen] > 0.0001 or injections[gen] < -0.0001:
                    logger.info("There exists atleast one non zero injection in current action")
                    return True
        
        logger.info("Current action will not cause any change")
        return False
    
    def _any_change_fire_state(self, fire_state):
        return self.previous_fire_state["node"] != fire_state["node"] or \
               self.previous_fire_state["branch"] != fire_state["branch"]

    def step(self, action, fire_state):
        self.protection_action_count = 0
        self.live_equipment_removal_penalty = 0
        action["generator_injection"] = np.around(action["generator_injection"], 4)  # handling floating point error

        if self._any_change_action(action) or self._any_change_fire_state(fire_state):
            logger.info("Current action will cause a change")
            has_violations  = self._check_violations(action)
            if not has_violations:
                self.has_converged = True
                self._check_protection_system_actions(fire_state)       # protection system operation counts
                self._check_live_line_removal_actions(action["branch_status"], action["bus_status"])    # live line removal operation counts

                self.previous_action = deepcopy(action)
                self.previous_fire_state = deepcopy(fire_state)
                self._solve_runtime_model() 
            else:
                self.has_converged = False
                logger.warn("Skipping this episode because of a violation")
        else:
            logger.info("Skipping this iteration as no change detected")
            pass
            
    def reset(self):
        self._remove_temp_files()
        self._initialize()
        return self.get_state()
        
    def _remove_temp_files(self):
        temp_files = glob.glob(os.path.join(self.gams_dir, '_gams_py_*'))
        logger.debug("Deleting {} files from directory {}".format(len(temp_files), self.gams_dir))
        for temp_file in temp_files:
            try:
                os.remove(temp_file)
            except:
                logger.warn("Not able to remove {}".format(temp_file))

    def get_load_loss(self):
        return round(sum(self.p_load_initial) - self.p_load_solved, 3)
    
    # def get_load_loss_weighted(self):
    #     non_critical_loss = sum(self.p_load_initial * (1- float(self.non_crtitcal_fractional[ctr_bus1][1]))) \
    #                         - sum(self.p_load_solved_distribution[1])
    #     critical_loss = sum(self.p_load_initial * float(self.non_crtitcal_fractional[ctr_bus1][1])) \
    #                     - sum(self.p_load_solved_distribution[0])
    #     weighted_load_loss = non_critical_loss*self.weights[1] + critical_loss*self.weights[0]
    #     return weighted_load_loss
    
    def get_status(self):
        return not self.has_converged
    
    def get_protection_operation_count(self):
        return self.protection_action_count
    
    def get_active_line_removal_count(self):
        return self.live_equipment_removal_count
    
    def get_active_line_removal_penalty(self):
        return self.live_equipment_removal_penalty
    
    @ staticmethod
    def mergeGenerators(ppc):
        ppc_gen_trim = []
        temp = ppc["gen"][0, :]
        ptr = 0
        ptr1 = 1
        while(ptr1 < ppc["gen"].shape[0]):
            if ppc["gen"][ptr, GEN_BUS] == ppc["gen"][ptr1, GEN_BUS]:
                temp[PG:QMIN+1] += ppc["gen"][ptr1, PG:QMIN+1]
                temp[PMAX:APF+1] += ppc["gen"][ptr1, PMAX:APF+1]
            else:
                ppc_gen_trim.append(temp)
                temp = ppc["gen"][ptr1, :]
                ptr = ptr1
            ptr1 += 1
        ppc_gen_trim.append(temp)
        ppc["gen"] = np.asarray(ppc_gen_trim)
    
    @ staticmethod
    def mergeBranches(ppc):
        ppc_branch_trim = []
        temp = ppc["branch"][0, :]
        ptr = 0
        ptr1 = 1
        while(ptr1 < ppc["branch"].shape[0]):
            if np.all(ppc["branch"][ptr, F_BUS:T_BUS+1] == ppc["branch"][ptr1, GEN_BUS:T_BUS+1]):
                temp[BR_R: RATE_C+1] += ppc["branch"][ptr1, BR_R: RATE_C+1]
            else:
                ppc_branch_trim.append(temp)
                temp = ppc["branch"][ptr1, :]
                ptr = ptr1
            ptr1 += 1
        ppc_branch_trim.append(temp)
        ppc["branch"] = np.asarray(ppc_branch_trim)

