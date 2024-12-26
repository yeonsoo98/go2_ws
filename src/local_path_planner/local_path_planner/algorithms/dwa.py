import math
from geometry_msgs.msg import Twist

class DWAAlgorithm:
    """
    간단한 DWA( Dynamic Window Approach ) 예시.
    실제 환경에선 추가 파라미터(가속도, 장애물충돌 체크, 세분화된 비용함수 등)를 더 고려해야 함.
    """

    def __init__(self):
        # 로봇 물리 제한
        self.max_speed = 0.5        # m/s (최대 선속도)
        self.min_speed = 0.0        # m/s (최소 선속도)
        self.max_yaw_rate = 1.0     # rad/s (최대 각속도)
        self.max_accel = 0.5        # m/s^2
        self.max_dyaw_rate = 1.0    # rad/s^2
        self.dt = 0.2               # 시뮬레이션 시간 스텝
        self.predict_time = 1.0     # 전방 시뮬레이션할 총 시간 (ex: 1초간 궤적)

        # 비용 함수 가중치
        self.heading_weight = 1.0
        self.speed_weight = 1.0
        self.clearance_weight = 1.2  # 장애물 회피 중요도

        # 장애물 판단 시 사용하는 costmap/occupancyGrid 또는 obstacle 좌표
        # 실제론 occupancyGrid를 참고하거나, 스캔 점들로 충돌체크
        self.obstacle_radius = 0.15  # 로봇 충돌 반경 (단순 가정)

    def compute_velocity(self, costmap, odom, goal):
        """
        costmap: OccupancyGrid (장애물 정보). 여기선 간소화해서 사용.
        odom: Odometry (현재 속도, 위치)
        goal: PoseStamped (목표 위치)

        returns: geometry_msgs/Twist
        """

        # 1) 현재 속도(선속도 v, 각속도 w) 추출
        current_v = math.hypot(odom.twist.twist.linear.x,
                               odom.twist.twist.linear.y)
        current_w = odom.twist.twist.angular.z

        # 2) DWA 동적 윈도 설정 → 샘플링할 속도 범위
        v_min = max(self.min_speed, current_v - self.max_accel*self.dt)
        v_max = min(self.max_speed, current_v + self.max_accel*self.dt)
        w_min = current_w - self.max_dyaw_rate*self.dt
        w_max = current_w + self.max_dyaw_rate*self.dt

        # 샘플링 스텝 (개수는 상황에 따라 조절)
        v_sample_num = 5
        w_sample_num = 5

        best_score = -99999.0
        best_v = 0.0
        best_w = 0.0

        # 3) 모든 (v,w) 후보에 대해 시뮬레이션
        for vi in self._frange(v_min, v_max, (v_max-v_min)/(v_sample_num-1 + 1e-9)):
            for wi in self._frange(w_min, w_max, (w_max-w_min)/(w_sample_num-1 + 1e-9)):

                # forward simulate
                score, collision = self.evaluate_trajectory(odom, costmap, vi, wi, goal)

                # 충돌 시 score= -∞ (또는 skip)
                if collision:
                    continue

                # 최고 점수를 갱신
                if score > best_score:
                    best_score = score
                    best_v = vi
                    best_w = wi

        # 4) 최적 (v, w)로 Twist 생성
        cmd = Twist()
        # 단순히 x축 선속도 = best_v, z축 각속도 = best_w
        cmd.linear.x = best_v
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = best_w
        return cmd

    def evaluate_trajectory(self, odom, costmap, v, w, goal):
        """
        (v, w) 후보로 dt~predict_time 동안 시뮬레이션,
        1) heading cost( goal까지의 거리 등 )
        2) speed cost
        3) clearance cost(장애물과의 거리)
        등을 종합해 score 계산
        만약 시뮬레이션 중 장애물 충돌이면 collision=True
        """
        # 로봇 초기 상태
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        # yaw
        q = odom.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0-2.0*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        collision = False
        time = 0.0
        dt_sim = 0.1  # 시뮬레이션 내부 스텝

        # 목표점
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        # 비용 계산용 변수
        final_x = x
        final_y = y
        clearance_min = 9999.0

        while time <= self.predict_time:
            # 운동학 모델: (x,y,yaw) 갱신
            x += v*dt_sim*math.cos(yaw)
            y += v*dt_sim*math.sin(yaw)
            yaw += w*dt_sim

            # 충돌체크
            # 여기선 예시로 costmap data 또는 obstacle array가 있다고 가정:
            if self.check_collision(x, y, costmap):
                collision = True
                break

            # clearance(장애물 최소 거리)
            dist2obs = self.estimate_clearance(x, y, costmap)
            if dist2obs < clearance_min:
                clearance_min = dist2obs

            final_x = x
            final_y = y

            time += dt_sim

        if collision:
            return (0.0, True)  # 충돌이면 점수 무효

        # heading cost: goal과 최종 위치 거리
        dist_to_goal = math.hypot(gx - final_x, gy - final_y)
        heading_score = 1.0 / (dist_to_goal + 0.001)

        # speed cost: 속도가 빠를수록 가중
        speed_score = v

        # clearance cost: 장애물과의 최소거리
        clearance_score = clearance_min

        # 가중합
        score = (self.heading_weight * heading_score
                 + self.speed_weight * speed_score
                 + self.clearance_weight * clearance_score)

        return (score, False)

    def check_collision(self, x, y, costmap):
        """
        간단 충돌 체크:
        - (x,y)를 costmap grid로 변환
        - 그 셀이 장애물(=100)인지 확인 (로봇 반경 약간 고려)
        실제론 footprint polygon collision check 필요
        """
        # base_link frame 이라면 -> costmap origin 고려
        # 여기선 odom frame이라고 가정할 수도. 코드 환경에 맞게 수정 필요.

        # (px, py) => grid (mx, my)
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        mx = int((x - origin_x)/resolution)
        my = int((y - origin_y)/resolution)

        # 범위체크
        if mx<0 or mx>=costmap.info.width or my<0 or my>=costmap.info.height:
            # 밖이면 일단 충돌이라 가정(또는 그냥 false return)
            return True

        idx = my*costmap.info.width + mx
        val = costmap.data[idx]

        # 100이면 장애물
        if val == 100:
            return True
        return False

    def estimate_clearance(self, x, y, costmap):
        """
        장애물까지의 최소거리 (단순 측정).
        여기서는 grid 주변 탐색 또는 radius check
        실제론 footprint를 이용한 full collision check를 해야 함.
        """

        # 여기서는 그냥 (x,y)에 가장 가까운 obstacle cell 거리 대충 계산
        # (비효율적이지만 예시용)
        min_dist = 9999.0
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        w = costmap.info.width
        h = costmap.info.height
        res = costmap.info.resolution

        # 샘플방식 (모든 cell 탐색) => 실제론 kd-tree나 distance transform 이용
        for iy in range(h):
            for ix in range(w):
                idx = iy*w + ix
                if costmap.data[idx] == 100:
                    # obstacle
                    ox = origin_x + (ix+0.5)*res
                    oy = origin_y + (iy+0.5)*res
                    dist = math.hypot(ox - x, oy - y)
                    if dist < min_dist:
                        min_dist = dist
        return min_dist

    def _frange(self, start, stop, step):
        """
        float 범위를 순회하는 유틸 함수 (Python range for floats)
        """
        r = start
        while r <= stop:
            yield r
            r += step
