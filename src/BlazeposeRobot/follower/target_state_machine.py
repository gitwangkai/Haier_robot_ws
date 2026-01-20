class TargetStateMachine:
    TRACKING = "tracking"
    LOST = "lost"
    STOPPED = "stopped"

    def __init__(self, lost_threshold=10):
        self.state = self.STOPPED
        self.lost_counter = 0
        self.lost_threshold = lost_threshold

    def update(self, has_target: bool):
        """
        has_target: 当前帧是否检测到人
        """
        if has_target:
            self.lost_counter = 0
            self.state = self.TRACKING
        else:
            self.lost_counter += 1
            if self.lost_counter >= self.lost_threshold:
                self.state = self.STOPPED
            else:
                self.state = self.LOST

        return self.state
