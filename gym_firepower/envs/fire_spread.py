from math import ceil, floor
import random
import json
from skimage.draw import line
import numpy as np
from collections import defaultdict
from copy import deepcopy
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
from gym import logger
from gym.utils import seeding
# from gym_firepower.envs.fire_spread_log_writer import FireSpreadInfoWriter
from fire_spread_log_writer import FireSpreadInfoWriter
from enum import Enum


DEFAULT_FUEL_TYPE = -3
DEFAULT_FUEL_AMT = 100
DEFAULT_SPREAD_PROBAB = 0.03


class CellState(Enum):
    UNBURNT = 0
    BURNING = 1
    BURNT = 2


class Grid(object):
    def __init__(self, geo_file, scaling_factor, rng):
        self.rng = rng
        self.total = 0
        self.scaling_factor = scaling_factor
        self.branch_ids = defaultdict(dict)
        self._parse_geo_file(geo_file)
        self._create_grid(rng)
        self._register_neighboring_cells()

    def _parse_geo_file(self, geo_file):
        args = {"cols": 40, "rows": 40, "sources": [[5, 5]], "seed": 30, "random_source":False, "num_sources":1}
        with open(geo_file, 'r') as config_file:
            args.update(json.load(config_file))

        self.rows = int(args["rows"])
        self.cols = int(args["cols"])
        self.state = np.full((self.rows, self.cols), CellState.UNBURNT)

        self.fuel_amt = np.full((self.rows, self.cols), DEFAULT_FUEL_AMT)
        self.fuel_type = np.full((self.rows, self.cols), DEFAULT_FUEL_TYPE)
        self.spread_probab = np.full((self.rows, self.cols), DEFAULT_SPREAD_PROBAB)
        self._update_matrices_based_on_configuration_file(args)

        self.random_source = args["random_source"]
        self.boxes = args["boxes"]
        self.num_sources = args["num_sources"]
        self.fire_source = np.full((self.rows, self.cols), False)

        self.sources = [ (arg[1], arg[0]) for arg in args["sources"] ]
        for src in self.sources:
            self.fire_source[src[0], src[1]] = True

        self.branches = args["branches"]
        self.critical_cells = self._identify_critical_cells(args["bus_ids"], args["branches"])

        self._create_base_image(args["fuel_type"])
        self.newly_added_burning_cells = np.array([cell for cell in self.sources], dtype=int)
        self._burning_cells = self.sources
        self.fire_distance = {"nodes": {}, "branches": {} }
        self._calculate_distance_from_fire()

    def _identify_critical_cells(self, bus_ids, branches):
        assert len(bus_ids) > 1, "There should be atleat 2 buses"
        cells = []
        if (bus_ids is not None) and (branches is not None):
            self.bus_ids = {}
            for bus in bus_ids:
                self.bus_ids[bus[0]] = (bus[2], bus[1])
                cells.append([bus[2], bus[1]])
            for branch in branches:
                x0 = self.bus_ids[branch[0]]
                x1 = self.bus_ids[branch[1]]
                points = line(x0[0], x0[1], x1[0], x1[1])
                points = np.column_stack((points[0], points[1]))
                cells.extend(points.tolist())
                self.branch_ids[branch[0]].update({branch[1]: points})
        return cells
    
    def _update_matrices_based_on_configuration_file(self, args):
        if "fuel_type" in args.keys():
            self._update_with_configuration(args["fuel_type"], self.fuel_type)
        if "fuel_amt" in args.keys():
            self._update_with_configuration(args["fuel_amt"], self.fuel_amt)
        if "spread_probab" in args.keys():
            self._update_with_configuration(args["spread_probab"], self.spread_probab)
    
    def _update_with_configuration(self, conf_sparse_matrix, full_matrix):
        for col, row, val in conf_sparse_matrix:
            col = int(col)
            row = int(row)
            full_matrix[row,col] = val
    
    def _create_grid(self, rng):
        self.grid = []
        for row in range(self.rows):
            temp = []
            for col in range(self.cols):
                params_dict = self._get_cell_params(row, col)
                params_dict.update({"rng": rng})
                temp.append(Cell(**params_dict))
            self.grid.append(temp)

    def _get_cell_params(self, row, col):
        params = {}
        params["row"] = row
        params["col"] = col
        params["scaling_factor"] = self.scaling_factor
        params["fuel_type"] = self.fuel_type[row,col]
        params["fuel_amt"] = self.fuel_amt[row,col]
        params["spread_probab"]= self.spread_probab[row,col]
        if (row,col) in self.sources:
            params["source_flag"] = True
            self.state[row, col] = CellState.BURNING
        else:
            params["source_flag"] = False
        return params
    
    def _register_neighboring_cells(self):
        delta_row_col = [ (-1,-1), (-1,0), (-1,1), (0,-1),
                            (0,1), (1,-1), (1, 0), (1,1) ]
        for row in range(self.rows):
            for col in range(self.cols):
                neighbors = []
                for delta in delta_row_col:
                    new_row = row + delta[0]
                    new_col = col + delta[1]
                    if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                        neighbors.append(self.grid[new_row][new_col])
                self.grid[row][col].register_neighbors(neighbors)
    
    def _create_base_image(self, fuel_type):
        self.base_image = np.zeros((self.rows, self.cols, 3), np.uint8)
        self.base_image[:, :, 1] = np.full((self.rows, self.cols), 255)
        for element in fuel_type:
            if element[2] == 0:
                self.base_image[element[1], element[0], 1] = 0
                self.base_image[element[1], element[0], 2] = 255
        for branch in self.branches:
            x0 = self.bus_ids[branch[0]]
            x1 = self.bus_ids[branch[1]]
            rr, cc = line(x0[0], x0[1], x1[0], x1[1])
            self.base_image[rr, cc, : ] = 10

    def get_current_image(self):
        temp = deepcopy(self.base_image)
        temp[:,:,0] = 100*self.state
        temp[:,:,1] = temp[:,:,1] - self.state*temp[:,:,1]
        return temp

    def step_ajay(self):
        self.newly_added_burning_cells = []
        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col].step_ajay()
                if self.grid[row][col].next_state != self.grid[row][col].state:
                    if self.grid[row][col].next_state == CellState.BURNING:
                        self.newly_added_burning_cells.append((row, col))
        self.newly_added_burning_cells = np.array(self.newly_added_burning_cells, dtype=int)

        for row in range(self.rows):
            for col in range(self.cols):
                self.state[row, col] = self.grid[row][col].update_state()

        # self.total += self.newly_added_burning_cells.size/2
        # print("total_burning_cells: ", int(self.total))

    def step(self):
        burnt_cells = []
        newly_added_burning_cells = []

        for cell in self._burning_cells:
            row, col = cell[0], cell[1]
            state = self.grid[row][col].step(newly_added_burning_cells)
            if state == CellState.BURNT:
                burnt_cells.append(cell)

        for burnt_cell in burnt_cells:
            self.state[burnt_cell] = CellState.BURNT
            self._burning_cells.remove(burnt_cell)

        for cell in newly_added_burning_cells:
            self.state[cell] = CellState.BURNING
        self._burning_cells = self._burning_cells + newly_added_burning_cells

        self.newly_added_burning_cells = np.array(newly_added_burning_cells, dtype=int)
        # self.total += self.newly_added_burning_cells.size/2
        # print("total_burning_cells:", int(self.total))

    def reset(self):
        self._get_new_sources()
        print("fire starts at: ", self.sources)

        for row in range(self.rows):
            for col in range(self.cols):
                src_flag = self.fire_source[row,col] 
                self.state[row, col] = self.grid[row][col].reset(src_flag)

        self.newly_added_burning_cells = np.array([cell for cell in self.sources], dtype=int)
        self._burning_cells = self.sources
        print("reset: burning_cells:", len(self._burning_cells))

        self.fire_distance = {"nodes": {}, "branches": {}}
        self._calculate_distance_from_fire()
    
    def _get_new_sources(self):
        if self.random_source:
            self.fire_source = np.full((self.rows, self.cols), False)
            self.sources = []
            if len(self.boxes) < 1:
                assert False, "at least one box should be defined"
            for box in self.boxes:
                num_sources = self.num_sources
                max_row = max(box[1][0], box[0][0])
                min_row = min(box[1][0], box[0][0])
                max_col = max(box[1][1], box[0][1])
                min_col = min(box[1][1], box[0][1])
                while (num_sources > 0):
                    row = self.rng.randint(min_row, max_row)
                    col = self.rng.randint(min_col, max_col)
                    if self.grid[row][col].fuel_type !=0 and [row, col] not in self.critical_cells:
                        self.sources.append((row, col))
                        self.fire_source[row][col] = True
                        num_sources -= 1

    def get_state(self):
        return deepcopy(self.state)
    
    def get_reduced_state(self):
        bus_dict = {}
        branch_dict = {}
        for bus in self.bus_ids:
            row, col = self.bus_ids[bus]
            state = self.state[row, col]
            if state == CellState.UNBURNT:
                bus_dict[bus] = 1 # 1 implies bus is in service
            else:
                bus_dict[bus] = 0 # 0 implies bus is out of service
        for branch in self.branches:
            from_bus = int(branch[0])
            to_bus = int(branch[1])
            cell_ids = self.branch_ids[from_bus][to_bus]
            answer = 1
            for cell_id in cell_ids:
                state = self.state[cell_id[0], cell_id[1]]
                if state != CellState.UNBURNT:
                    answer = 0
                    break
            branch_dict[(from_bus, to_bus)] = answer
        return {"node": deepcopy(bus_dict), "branch": deepcopy(branch_dict)}
    
    def get_distance_from_fire(self):
        self._calculate_distance_from_fire()
        return deepcopy(self.fire_distance)

    def _calculate_distance_from_fire(self):
        if self.newly_added_burning_cells.shape[0] > 0:
            bus_dict = self.fire_distance["nodes"]
            branch_dict = self.fire_distance["branches"]
            for bus in self.bus_ids:
                bus_idx = np.array(self.bus_ids[bus])
                min_dist = min([round(np.linalg.norm(cell - bus_idx), 3) for cell in self.newly_added_burning_cells])
                try:
                    if bus_dict[bus] > min_dist:
                        bus_dict[bus] = min_dist
                except KeyError:
                    bus_dict[bus] = min_dist
                
            for branch in self.branches:
                from_bus = int(branch[0])
                to_bus = int(branch[1])
                min_dist = min([self._calculate_distance(self.bus_ids[from_bus], self.bus_ids[to_bus], cell)
                                for cell in self.newly_added_burning_cells])
                try:
                    if branch_dict[(from_bus, to_bus)] > min_dist:
                        branch_dict[(from_bus, to_bus)] = min_dist
                except KeyError:
                    branch_dict[(from_bus, to_bus)] = min_dist
            
    def _calculate_distance(self, p1, p2, P):
        """ segment line AB, point P, where each one is an array([x, y]) """
        A = np.array(p1)
        B = np.array(p2)
        if all(A == P) or all(B == P):
            return 0
        if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > pi / 2:
            return norm(P - A)
        if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > pi / 2:
            return norm(P - B)
        return round(norm(cross(A-B, A-P))/norm(B-A),3)


class Cell(object):
    def __init__(self, row, col, fuel_type, fuel_amt, spread_probab,
                    scaling_factor, source_flag, rng):
        self.rng = rng
        self.row = row
        self.col = col
        '''
        fuel type defines the slope of the fuel consumption curve i.e. fuel consumption rate
        fuel consumption curve x(k+1) = a* x(k) + b where a <= 0
        a = 0 implies the fuel cannot be burnt
        '''
        assert fuel_type <= 0, "incorrect fuel type, slope should be non positive"
        self.fuel_type = fuel_type
        assert fuel_amt >= 0, "fuel amount cannot be negative"
        assert (not (fuel_type < 0) or (fuel_amt > 0)), "Fuel amount should be more \
                                            than 0 if the type of cell is flammable" 

        self.init_amt = fuel_amt
        self.spread_probab = spread_probab     # probability of spreading fire from cell x to each neighbor.
        self.scaling_factor = scaling_factor   # determines number of steps of fire model == number of steps power model

        self.neighbors = []
        self.fuel_amount = self.init_amt
        self.state = CellState.UNBURNT
        self.next_state = CellState.UNBURNT
        self.counter = 1
        self.source_flag = source_flag
        self.set_source()

    def register_neighbors(self, neighbors):
        self.neighbors = neighbors
    
    def step_ajay(self):
        if self.fuel_type != 0:
            if self.state == CellState.UNBURNT:
                if self.counter == self.scaling_factor:
                    self.counter = 1
                    pho = 1
                    for neighbor in self.neighbors:           #probability increases if more neighbors in burning state
                        if neighbor.state == CellState.BURNING:
                            pho *= 1 - neighbor.spread_probab
                    pho = 1 - pho
                    
                    roll_dice = self.rng.uniform(0, 1)
                    if roll_dice <= pho:
                        self.next_state = CellState.BURNING
                else:
                    self.counter += 1
            elif self.state == CellState.BURNING:
                self.fuel_amount = self.fuel_amount + self.fuel_type
                if self.fuel_amount <= 0:
                    self.next_state = CellState.BURNT
            elif self.state == CellState.BURNT:
                pass

    def step(self, newly_added_burning_cells):
        for neighbor in self.neighbors:
            if neighbor.fuel_type != 0 and neighbor.state == CellState.UNBURNT:
                roll_dice = self.rng.uniform(0, 1)
                if roll_dice <= self.spread_probab:
                    neighbor.state = CellState.BURNING
                    newly_added_burning_cells.append((neighbor.row, neighbor.col))

        self.fuel_amount = self.fuel_amount + self.fuel_type
        if self.fuel_amount <= 0:
            self.state = CellState.BURNT
        return self.state

    def get_state(self):
        return self.state

    def reset(self, source_flag):
        self.fuel_amount = self.init_amt
        self.state = CellState.UNBURNT
        self.next_state = CellState.UNBURNT
        self.counter = 1
        self.source_flag = source_flag
        self.set_source()
        return self.state

    def update_state(self):
        self.state = self.next_state
        return self.state
    
    def set_source(self):
        if self.source_flag:
            assert self.fuel_type != 0, "nonflammable cell cannot be set up as fuel"
            self.state = CellState.BURNING
            self.next_state = CellState.BURNING


class FireSpread(object):
    def __init__(self, conf_file, factor, seed, save_fire_spread_info=False):
        np_rng, seed = seeding.np_random(seed)
        self.grid = Grid(conf_file, factor, np_rng)

        self._save_fire_spread_info = save_fire_spread_info
        if self._save_fire_spread_info:
            self.fire_stats_writer = FireSpreadInfoWriter("./", seed, DEFAULT_SPREAD_PROBAB)

    def get_state(self):
        return self.grid.get_state()

    def get_reduced_state(self):
        fire_state = self.grid.get_reduced_state()
        if self._save_fire_spread_info:
            self.fire_stats_writer.add_info(fire_state)
        return fire_state

    def step(self):
        self.grid.step()

    def reset(self):
        self.grid.reset()
        if self._save_fire_spread_info:
            self.fire_stats_writer.reset()

    def get_current_image(self):
        return self.grid.get_current_image()

    def get_distance_from_fire(self):
        return self.grid.get_distance_from_fire()


if __name__ == "__main__":
    conf_file = "./../../../FirePower-agent-private/configurations/configuration.json"
    seed = 50
    fire_spread = FireSpread(conf_file, 1, seed, True)

    num_of_episode = 10
    num_of_steps = 300
    for j in range(num_of_episode):
        fire_spread.reset()

        for i in range(num_of_steps):
            fire_spread.step()
            state = fire_spread.get_reduced_state()
            print("episode:", j, ", step:", i, "state:", state["branch"])

