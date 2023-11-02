class TimeKeeper:
    def __init__(self, dt) -> None:
        self.dt = dt
        self.step_count = 0
        self.time_stamp = 0

    def tick(self):
        self.step_count += 1
        self.time_stamp += self.dt
