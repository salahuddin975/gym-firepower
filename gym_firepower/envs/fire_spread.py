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

DEFAULT_FUEL_TYPE = -3
DEFAULT_FUEL_AMT = 100
DEFAULT_SPREAD_PROBAB = 0.3

class Grid(object):
    def __init__(self, geo_file, scaling_factor, rng):
        self.rng = rng
        self.scaling_factor = scaling_factor
        self.branch_ids = defaultdict(dict)
        self._parse_geo_file(geo_file)
        self._create_grid(rng)
        self._register_neighboring_cells()

    def _parse_geo_file(self, geo_file):
        args = {"cols": 40, "rows": 40, "sources": [[5, 5]], 
                "seed": 30, "random_source":False, "num_sources":1}

        with open(geo_file, 'r') as config_file:
            args.update(json.load(config_file))

        self.rows = int(args["rows"])
        self.cols = int(args["cols"])
        self.random_source = args["random_source"]
        self.boxes = args["boxes"]
        self.num_sources = args["num_sources"]
        self.fuel_amt = np.full((self.rows, self.cols), DEFAULT_FUEL_AMT)
        self.fuel_type = np.full((self.rows, self.cols), DEFAULT_FUEL_TYPE)
        self.spread_probab = np.full((self.rows, self.cols), DEFAULT_SPREAD_PROBAB)
        self.fire_source = np.full((self.rows, self.cols), False)
        self.state = np.full((self.rows, self.cols), 0)

        self.sources = [ (arg[1], arg[0]) for arg in args["sources"] ]
        for src in self.sources:
            self.fire_source[src[0], src[1]] = True

        self.branches = args["branches"]
        self.critical_cells = self._identify_critical_cells(
            args["bus_ids"], args["branches"])
        self._create_matrices(args)
        self._create_base_image(args["fuel_type"])
        self.burning_cells = np.array([ cell for cell in self.sources], dtype=int)
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
    
    def _create_matrices(self, args):
        if "fuel_type" in args.keys():
            self._create_matrix(args["fuel_type"], self.fuel_type)
        if "fuel_amt" in args.keys():
            self._create_matrix(args["fuel_amt"], self.fuel_amt)
        if "spread_probab" in args.keys():
            self._create_matrix(args["spread_probab"], self.spread_probab)
    
    def _create_matrix(self, sparse_matrix, full_matrix):
        for col, row, val in sparse_matrix:
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
            self.state[row, col] = 1
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
                    if 0 <= new_row < self.rows and \
                        0 <= new_col < self.cols:
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
        # temp[:,:,0] = temp[:,:,0] + 50*self.state
        temp[:,:,0] = 100*self.state
        temp[:,:,1] = temp[:,:,1] - self.state*temp[:,:,1]
        return temp

    def step(self):
        self.burning_cells = []
        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col].step()
                if self.grid[row][col].state_ != self.grid[row][col].state:
                    if self.grid[row][col].state_ == 1:
                        self.burning_cells.append((row,col))
        self.burning_cells = np.array(self.burning_cells, dtype=int)
        
        for row in range(self.rows):
            for col in range(self.cols):
                self.state[row, col] = self.grid[row][col].update_state()
                
    def reset(self):
        self._get_new_sources()
        print("fire starts at: ", self.sources)
        for row in range(self.rows):
            for col in range(self.cols):
                src_flag = self.fire_source[row,col] 
                self.state[row, col] = self.grid[row][col].reset(src_flag)
        self.burning_cells = np.array([cell for cell in self.sources], dtype=int)
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
                sources = []
                max_row = max(box[1][0], box[0][0])
                min_row = min(box[1][0], box[0][0])
                max_col = max(box[1][1], box[0][1])
                min_col = min(box[1][1], box[0][1])
                while (num_sources > 0):
                    row = self.rng.randint(min_row, max_row)
                    col = self.rng.randint(min_col, max_col)
                    if self.grid[row][col].fuel_type !=0 and \
                        [row, col] not in self.critical_cells:
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
            if state == 0:
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
                if state != 0:
                    answer = 0
                    break
            branch_dict[(from_bus, to_bus)] = answer
        return {"node": deepcopy(bus_dict), "branch": deepcopy(branch_dict)}
    
    def get_distance_from_fire(self):
        self._calculate_distance_from_fire()
        return deepcopy(self.fire_distance)

    def _calculate_distance_from_fire(self):
        if self.burning_cells.shape[0] > 0:
            bus_dict = self.fire_distance["nodes"]
            branch_dict = self.fire_distance["branches"]
            for bus in self.bus_ids:
                bus_idx = np.array(self.bus_ids[bus])
                min_dist = min([round(np.linalg.norm(cell - bus_idx), 3)
                                for cell in self.burning_cells])
                try:
                    if bus_dict[bus] > min_dist:
                        bus_dict[bus] = min_dist
                except KeyError:
                    bus_dict[bus] = min_dist
                
            for branch in self.branches:
                from_bus = int(branch[0])
                to_bus = int(branch[1])
                min_dist = min([ self._calculate_distance(
                    self.bus_ids[from_bus], self.bus_ids[to_bus], cell) 
                    for cell in self.burning_cells])
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
        '''
        spread prob, P(x-> neghbors) defines the probability of spreading fire from 
        cell x to each neighbor.
        '''
        self.spread_probab = spread_probab
        '''
        scaling factor determines the step size of the fire-spread model wrt power
        system or agents step size
        Assumption: the time step of fire spread is greater than power system step
        size and is a integer multiple
        '''
        self.scaling_factor = scaling_factor
        assert scaling_factor > 0 and isinstance(scaling_factor, int), \
        "Scaling factor should be an integer greater than 0"
        
        self.neighbors = []
        # transition probability to move from not burning to burning
        self.pho = 0
        self.B = False
        self.F = self.init_amt
        self.state = 0
        self.state_ = 0
        self.counter = 1
        self.source_flag = source_flag
        self.set_source()
        # print("Cell ({},{}) has spread probability of {}".format(self.row, self.col, self.spread_probab))

    def register_neighbors(self, neighbors):
        self.neighbors = neighbors
    
    def step(self):
        if self.fuel_type != 0:

            if self.state == 0: # 0 -> not burning
                assert self.F == self.init_amt, \
                    "Cannot be in this state if fuel is less than max"
                assert not self.B, "Cannot be in this state if B is True" 
                if self.counter == self.scaling_factor:
                    self.counter = 1
                    pho = 1
                    for neighbor in self.neighbors:
                        # only burning neighbors can contribute
                        if neighbor.state == 1 : 
                            pho *= 1 - neighbor.spread_probab
                    self.pho = 1 - pho
                    
                    roll_dice = self.rng.uniform(0, 1)
                    if roll_dice <= self.pho:
                        self.B = True
                        self.state_ = 1
                    else:
                        self.B = False
                    # print("Cell ({}, {}) rolled a dice with rho {} and got {}".format(self.row, self.col, self.pho, roll_dice))
                else:
                    self.counter += 1

            elif self.state == 1: # 1 -> burning 
                # print("Cell ({}, {}) is burning".format(self.row, self.col))
                assert self.B, " Cannot be in this state if B is False"
                assert 0 < self.F <= self.init_amt, \
                    "Cannot be in this state if fuel is less than 0"
                self.F = self.F + self.fuel_type
                if self.F <= 0:
                    self.state_ = 2
            
            elif self.state == 2: # 2 -> burnt
                pass
            
            else:
                assert False, "Cannot reach here"
        
    def reset(self, source_flag):
        self.pho = 0
        self.B = False
        self.F = self.init_amt
        self.state = 0
        self.state_ = 0
        self.counter = 1
        self.source_flag = source_flag
        self.set_source()
        return self.state

    def get_state(self):
        return self.state
    
    def update_state(self):
        self.state = self.state_
        return self.state
    
    def set_source(self):
        if self.source_flag:
            assert self.fuel_type != 0, "nonflammable cell cannot be set up as fuel"
            self.B =  True
            self.state = 1
            self.state_ = 1

    def update_source_flag(self, val):
        self.source_flag = val
    
class FireSpread(object):
    def __init__(self, conf_file, factor, rng):
        self.grid = Grid(conf_file, factor, rng)
    
    def get_state(self):
        return self.grid.get_state()
    
    def get_reduced_state(self):
        return self.grid.get_reduced_state()
    
    def step(self):
        self.grid.step()
    
    def reset(self):
        self.grid.reset()

    def get_current_image(self):
        return self.grid.get_current_image()
    
    def get_distance_from_fire(self):
        return self.grid.get_distance_from_fire()
