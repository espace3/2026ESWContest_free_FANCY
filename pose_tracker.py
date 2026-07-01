class PoseTracker:
    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self.smooth = {
            k: {"cx": 0.5, "cy": 0.5, "visible": False}
            for k in ("head", "upper", "lower")
        }
        self.miss_count = 0
        self.MISS_THR = 5

    def update(self, result):
        if result["detected"]:
            self.miss_count = 0
        else:
            self.miss_count += 1

        for name, r in result["regions"].items():
            s = self.smooth[name]
            if r["visible"]:
                s["cx"] = self.alpha * r["cx"] + (1 - self.alpha) * s["cx"]
                s["cy"] = self.alpha * r["cy"] + (1 - self.alpha) * s["cy"]
                s["visible"] = True
            # visible 아니어도 좌표 유지

        if self.miss_count >= self.MISS_THR:
            for s in self.smooth.values():
                s["visible"] = False

        return self.smooth