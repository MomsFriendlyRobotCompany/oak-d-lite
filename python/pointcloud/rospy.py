from collections import deque

class Time:
    @staticmethod
    def from_sec(sec):
        return sec

# Publisher("/camera/imu", Imu, queue_size=50)
class Publisher:
    def __init__(self, key, type, queue_size=0):
        self.data = deque()
        self.key = key

    def __del__(self):
        print(f"{self.key}: {len(self.data)}")

    def publish(self, info):
        self.data.append(info)

    def __iter__(self):
        return self

    def __next__(self):
        for d in self.data:
            yield d