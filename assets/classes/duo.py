class Duo:
    def __init__(self, sensor_left, sensor_right, sensor_back=None):
        self.left = sensor_left
        self.right = sensor_right

        if sensor_back is not None:
            self.back = sensor_back

        self.values = (self.left, self.right)