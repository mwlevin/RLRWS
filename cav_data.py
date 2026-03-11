class CavData:
    def __init__(self, predicted_state, loc, spd):
        self.predicted_state = predicted_state
        self.loc = loc
        self.spd = spd

class PredData:
    def __init__(self, pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, spd_pred_ego, predicted_tl_state, pos_ego, spd_ego):
        self.pos_pred_ego = pos_pred_ego
        self.pos_pred_max_ego = pos_pred_max_ego
        self.pos_pred_min_ego = pos_pred_min_ego
        self.spd_pred_ego = spd_pred_ego
        self.predicted_tl_state = predicted_tl_state
        self.pos_ego = pos_ego
        self.spd_ego = spd_ego