import os
import csv


class FireSpreadInfoWriter:
    def __init__(self, base_path, seed, spread_prob):
        self._seed = seed
        self._spread_prob = spread_prob

        self._dir_name = os.path.join(base_path, "fire_stats")
        self._node_status = os.path.join(self._dir_name, "node_status")
        self._branch_status = os.path.join(self._dir_name, "branch_status")

        self._create_dir()
        self._initialize_node_info()
        self._initialize_branch_info()

        self.episode = -1
        self.step = 0

    def _create_dir(self):
        try:
            os.makedirs(self._dir_name)
        except OSError as error:
            print(error)

    def _initialize_node_info(self):
        with open(f'{self._node_status}_seed_{self._seed}_sp_{self._spread_prob}.csv', 'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(["episode", "step", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13",
                             "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24"])

    def _initialize_branch_info(self):
        with open(f'{self._branch_status}_seed_{self._seed}_sp_{self._spread_prob}.csv', 'w') as fd:
            writer = csv.writer(fd)
            writer.writerow(["episode", "step", "[1, 2]", "[1, 3]", "[1, 5]", "[2, 4]", "[2, 6]", "[3, 9]", "[3, 24]",
                             "[4, 9]", "[5, 10]", "[6, 10]", "[7, 8]", "[8, 9]", "[8, 10]", "[9, 11]", "[9, 12]", "[10, 11]",
                             "[10, 12]", "[11, 13]", "[11, 14]", "[12, 13]", "[12, 23]", "[13, 23]", "[14, 16]",
                             "[15, 16]", "[15, 21]", "[15, 24]", "[16, 17]", "[16, 19]", "[17, 18]",
                             "[17, 22]", "[18, 21]", "[19, 20]", "[20, 23]", "[21, 22]"
                             ])

            # writer.writerow(["episode", "step", "[0, 1]", "[0, 2]", "[0, 4]", "[1, 3]", "[1, 5]", "[2, 8]", "[2, 23]",
            #                  "[3, 8]", "[4, 9]", "[5, 9]", "[6, 7]", "[7, 8]", "[7, 9]", "[8, 10]", "[8, 11]", "[9, 10]",
            #                  "[9, 11]", "[10, 12]", "[10, 13]", "[11, 12]", "[11, 22]", "[12, 22]", "[13, 15]",
            #                  "[14, 15]", "[14, 20]", "[14, 20]", "[14, 23]", "[15, 16]", "[15, 18]", "[16, 17]",
            #                  "[16, 21]", "[17, 20]", "[17, 20]", "[18, 19]", "[18, 19]", "[19, 22]", "[19, 22]", "[20, 21]"
            #                  ])

    def reset(self):
        self.step = 0
        self.episode += 1

    def add_info(self, info):
        self._add_node_info(info["node"])
        self._add_branch_info(info["branch"])
        self.step += 1

    def _add_node_info(self, ni):
        with open(f'{self._node_status}_seed_{self._seed}_sp_{self._spread_prob}.csv', 'a') as fd:
            writer = csv.writer(fd)
            writer.writerow([str(self.episode), str(self.step), ni[0], ni[1], ni[2], ni[3], ni[4], ni[5], ni[6], ni[7],
                             ni[8], ni[9], ni[10], ni[11], ni[12], ni[13], ni[14], ni[15], ni[16], ni[17], ni[18],
                             ni[19], ni[20], ni[21], ni[22], ni[23]])

    def _add_branch_info(self, bi):
        with open(f'{self._branch_status}_seed_{self._seed}_sp_{self._spread_prob}.csv', 'a') as fd:
            writer = csv.writer(fd)
            writer.writerow([str(self.episode), str(self.step), bi[(0, 1)], bi[(0, 2)], bi[(0, 4)], bi[(1, 3)],
                             bi[(1, 5)], bi[(2, 8)], bi[(2, 23)], bi[(3, 8)], bi[(4, 9)], bi[(5, 9)],
                             bi[(6, 7)], bi[(7, 8)], bi[(7, 9)], bi[(8, 10)], bi[(8, 11)], bi[(9, 10)],
                             bi[(9, 11)], bi[(10, 12)], bi[(10, 13)], bi[(11, 12)], bi[(11, 22)],
                             bi[(12, 22)], bi[(13, 15)], bi[(14, 15)], bi[(14, 20)], bi[(14, 23)],
                             bi[(15, 16)], bi[(15, 18)], bi[(16, 17)], bi[(16, 21)], bi[(17, 20)],
                             bi[(18, 19)], bi[(19, 22)], bi[(20, 21)]])

