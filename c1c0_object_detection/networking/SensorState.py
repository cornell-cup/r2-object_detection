from typing import List, Dict


class SensorState:
    """
    Class to keep track of the state of the sensor inputs for C1C0, to be stored on Jetson
        Instance Attributes:
            lidar (List[tuple[int, int]]): List of lidar values, each element is an (ang,dist) tuple
            terabee_bot (List[int]): List of bottom terabee distances, each element is distance
            terabee_top (List[int]): List of top terabee distances, each element is distance
            pos_x (int): X position of hedgehog in GPS sub-map
            pos_y (int): Y position of hedgehog in GPS sub-map
        Class Attributes:
            terabee_bot_ang (Dict[int, int]): mapping from indices in terabee array's to angle of reading
            terabee_top_ang (Dict[int, int]): mapping from indices in terabee array's to angle of reading
    """
    terabee_bot_ang: Dict[int, int] = {}
    terabee_top_ang: Dict[int, int] = {}

    def __init__(self):
        # initialize to max-size values for socket bytesize testing
        self.lidar: List[tuple[int, int]] = [1000]*360
        self.terabee_bot: List[int] = [1]*20
        self.terabee_mid: List[int] = [1]*20
        self.terabee_top: List[int] = [1]*20
        self.heading: int = 0

    def package_data(self):
        return [self.terabee_bot, self.terabee_mid, self.terabee_top, self.lidar]

    def set_lidar(self, data):
        self.lidar= [None]*360
        for i, j in data:
            self.lidar[i] = j

    def get_lidar(self):
        ans= []
        for i, j in enumerate(self.lidar):
            if j==None:
                continue
            ans.append((i,j))
        return ans

    def __str__(self):
        return "lidar: "+str(self.lidar) + "t_b: "+str(self.terabee_bot)

    def update(self) -> None:
        """
        Update function to read the serial lines and update the sensor state
        """
        pass
