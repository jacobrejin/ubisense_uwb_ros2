class Message:
    def __init__(self, tag, timestamp, x, y, z, variance):
        self.tag = tag
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.variance = variance