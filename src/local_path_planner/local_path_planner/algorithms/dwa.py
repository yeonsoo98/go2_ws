import math
from geometry_msgs.msg import Twist

class DWAAlgorithm:
    """
    간단한 DWA( Dynamic Window Approach ) 예시.
    실제 환경에선 추가 파라미터(가속도, 장애물충돌 체크, 세분화된 비용함수 등)를 더 고려해야 함.
    """

    def __init__(self):
        # 로봇 물리 제한
        self.max_speed = 0.4        # m/s (최대 선속도)
        self.min_speed = 0.0        # m/s (최소 선속도)
        self.max_yaw_rate = 1.0     # rad/s (최대 각속도)
        self.max_accel = 0.3        # m/s^2
        self.max_dyaw_rate = 1.0    # rad/s^2
        self.dt = 0.2               # 시뮬레이션 시간 스텝
        self.predict_time = 1.0     # 전방 시뮬레이션할 총 시간 (ex: 1초간 궤적)

        # 비용 함수 가중치
        self.heading_weight = 1.5
        self.speed_weight = 1.0
        self.clearance_weight = 2.0  # 장애물 회피 중요도
        self.obstacle_radius = 0.5  # 로봇 충돌 반경 

    def compute_velocity(self, costmap, odom, goal):
        # 1) 현재 속도(선속도 v, 각속도 w) 추출
        current_v = math.hypot(odom.twist.twist.linear.x,
                               odom.twist.twist.linear.y)
        current_w = odom.twist.twist.angular.z

        v_min = max(self.min_speed, current_v - self.max_accel*self.dt)
        v_max = min(self.max_speed, current_v + self.max_accel*self.dt)
        w_min = current_w - self.max_dyaw_rate*self.dt
        w_max = current_w + self.max_dyaw_rate*self.dt

        # 샘플링 스텝 (개수는 상황에 따라 조절)
        v_sample_num = 10
        w_sample_num = 10
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0

        for v in self._frange(v_min, v_max, (v_max - v_min) / (v_sample_num - 1)):
            for w in self._frange(w_min, w_max, (w_max - w_min) / (w_sample_num - 1)):
                score, collision = self.evaluate_trajectory(odom, costmap, v, w, goal)
                if collision:
                    continue
                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        return cmd


    def evaluate_trajectory(self, odom, costmap, v, w, goal):
        x, y, yaw = odom.pose.pose.position.x, odom.pose.pose.position.y, self._get_yaw(odom)
        gx, gy = goal.pose.position.x, goal.pose.position.y
        clearance_min = float('inf')
        time = 0.0
        dt_sim = 0.1
        collision = False

        while time <= self.predict_time:
            x += v * dt_sim * math.cos(yaw)
            y += v * dt_sim * math.sin(yaw)
            yaw += w * dt_sim

            if self.check_collision(x, y, costmap):
                collision = True
                break

            dist_to_obs = self.estimate_clearance(x, y, costmap)
            clearance_min = min(clearance_min, dist_to_obs)

            time += dt_sim

        if collision:
            return 0.0, True

        dist_to_goal = math.hypot(gx - x, gy - y)
        heading_score = 1.0 / (dist_to_goal + 1e-5)
        speed_score = v
        clearance_score = clearance_min

        score = (self.heading_weight * heading_score +
                 self.speed_weight * speed_score +
                 self.clearance_weight * clearance_score)

        return score, False
    
    def check_collision(self, x, y, costmap):
        origin_x, origin_y = costmap.info.origin.position.x, costmap.info.origin.position.y
        res = costmap.info.resolution
        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)

        if mx < 0 or mx >= costmap.info.width or my < 0 or my >= costmap.info.height:
            return True

        idx = my * costmap.info.width + mx
        return costmap.data[idx] == 100

    def estimate_clearance(self, x, y, costmap):
        origin_x, origin_y = costmap.info.origin.position.x, costmap.info.origin.position.y
        res = costmap.info.resolution
        w, h = costmap.info.width, costmap.info.height
        min_dist = float('inf')

        for iy in range(h):
            for ix in range(w):
                idx = iy * w + ix
                if costmap.data[idx] == 100:
                    ox = origin_x + (ix + 0.5) * res
                    oy = origin_y + (iy + 0.5) * res
                    dist = math.hypot(ox - x, oy - y)
                    min_dist = min(min_dist, dist)

        return min_dist

    def _get_yaw(self, odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

    def _frange(self, start, stop, step):
        r = start
        while r <= stop:
            yield r
            r += step