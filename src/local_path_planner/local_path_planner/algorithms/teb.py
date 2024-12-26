import math
import numpy as np
from geometry_msgs.msg import Twist

class TEBAlgorithm:
    """
    간단한 'Time Elastic Band' 모사 예시.
    (실제로는 C++ 기반 g2o 최적화, trajectory time-parameterization, collision checks 등 훨씬 복잡)
    """

    def __init__(self):
        # 로봇 제한
        self.max_vel_x = 0.5
        self.max_vel_theta = 1.0
        self.acc_lim_x = 0.5
        self.acc_lim_theta = 1.0

        # 최적화 파라미터 (예시)
        self.weight_obstacle = 1.2
        self.weight_goal = 1.0
        self.weight_viapoint = 0.5
        self.dt_ref = 0.2     # 기준 시간 간격
        self.n_poses = 10     # 간단히 10개 pose(trajectory state)로 구성

    def compute_velocity(self, costmap, odom, goal):
        """
        1) 현재 로봇 상태 + goal 기반 초기 trajectory 생성
        2) 반복 최적화(목표 접근, 장애물 회피, 속도 한계)
        3) 최종 첫 구간 속도 반환
        """
        # 로봇 현재 상태
        rx, ry, ryaw, rv, rw = self.get_robot_state(odom)

        # 1) 초기 trajectory (n_poses개)
        #    x[0] = (rx, ry, yaw=ryaw, t=0)
        #    x[-1] ~ (goal_x, goal_y, yaw=??, t ~ n_poses*dt_ref)
        #    가운데 점들: 직선 보간(단순)
        gx, gy = goal.pose.position.x, goal.pose.position.y
        init_trajectory = self.init_trajectory(rx, ry, ryaw, gx, gy)

        # 2) 반복 최적화 (간단히 single iteration 예시)
        optimized_trajectory = self.optimize_trajectory(init_trajectory,
                                                        costmap, 
                                                        (rx, ry, ryaw),
                                                        (gx, gy))

        # 3) 최종 첫 구간 속도 계산
        #    trajectory[0]이 현재, trajectory[1]이 다음 목표
        if len(optimized_trajectory) < 2:
            # fallback
            cmd = Twist()
            return cmd

        # 첫번 째 pose = (rx, ry, yaw, t=0)
        # 두번 째 pose = (nx, ny, nyaw, t=dt)
        (nx, ny, nyaw, t1) = optimized_trajectory[1]
        dx = nx - rx
        dy = ny - ry

        desired_yaw = math.atan2(dy, dx)
        dist = math.hypot(dx, dy)
        dt = self.dt_ref  # 임시

        # v ~ dist / dt
        v_des = dist / dt
        # w ~ yaw diff / dt
        yaw_diff = self.normalize_angle(desired_yaw - ryaw)
        w_des = yaw_diff / dt

        # 한계 적용
        v_des = max(min(v_des, self.max_vel_x), 0.0)
        w_des = max(min(w_des, self.max_vel_theta), -self.max_vel_theta)

        # 결과 Twist
        cmd = Twist()
        cmd.linear.x = v_des
        cmd.angular.z = w_des
        return cmd

    # ---------------------------
    # Trajectory init/optimization
    # ---------------------------
    def init_trajectory(self, rx, ry, ryaw, gx, gy):
        """
        n_poses개로 구성된 간단 line interpolation:
        t=0 ~ t_end, x,y는 선분 보간, yaw=도착각 적당히
        """
        traj = []
        # 종착 yaw
        goal_yaw = math.atan2(gy - ry, gx - rx)
        for i in range(self.n_poses + 1):
            alpha = i / float(self.n_poses)
            x_ = rx + alpha*(gx - rx)
            y_ = ry + alpha*(gy - ry)
            yaw_ = ryaw + alpha*self.normalize_angle(goal_yaw - ryaw)
            t_ = alpha * (self.n_poses * self.dt_ref)
            traj.append((x_, y_, yaw_, t_))
        return traj

    def optimize_trajectory(self, trajectory, costmap, start, goal_xy):
        """
        단순 1회 최적화 예시:
          1) 각 pose에 대해 grad 계산(목표와 거리, 장애물 cost)
          2) pose update
        실제 TEB는 수차례 반복, time re-parameterization 등
        """
        new_traj = []
        for i, (x, y, yaw_, t_) in enumerate(trajectory):
            # skip start/end
            if i==0 or i==len(trajectory)-1:
                new_traj.append((x, y, yaw_, t_))
                continue

            # compute gradient from goal
            gx, gy = goal_xy
            dist_goal = math.hypot(gx - x, gy - y)
            dgx = (x - gx)/ (dist_goal + 1e-9)  # gradient in x
            dgy = (y - gy)/ (dist_goal + 1e-9)

            # obstacle cost (가장 가까운 장애물, 대충)
            obst_dx, obst_dy = self.obstacle_gradient(x, y, costmap)
            
            # 합성 gradient
            grad_x = (self.weight_goal * dgx
                      + self.weight_obstacle * obst_dx)
            grad_y = (self.weight_goal * dgy
                      + self.weight_obstacle * obst_dy)

            step_size = 0.02
            newx = x - step_size * grad_x
            newy = y - step_size * grad_y
            # yaw는 단순 유지 (실제론 obstacle 각도도 반영해야)
            newyaw = yaw_

            new_traj.append((newx, newy, newyaw, t_))

        return new_traj

    # ---------------------------
    # 유틸 함수
    # ---------------------------
    def get_robot_state(self, odom):
        """
        odom에서 (x,y,yaw,v,w) 추출
        """
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0-2.0*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        v = math.hypot(vx, vy)
        w = odom.twist.twist.angular.z
        return (px, py, yaw, v, w)

    def obstacle_gradient(self, x, y, costmap):
        """
        장애물 costmap에서 (x,y)에 대한 gradient.
        매우 단순: obstacle=100인 지점이 가까우면, (dx,dy)↑
        """
        # brute force: 모든 obstacle cell과 거리, 그중 가장 가까운 것
        w = costmap.info.width
        h = costmap.info.height
        res = costmap.info.resolution
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y

        min_dist = 9999.0
        closest_ix, closest_iy = None, None

        for iy in range(h):
            for ix in range(w):
                idx = iy*w + ix
                if costmap.data[idx] == 100:  # obstacle
                    cell_x = ox + (ix+0.5)*res
                    cell_y = oy + (iy+0.5)*res
                    dist_ = math.hypot(cell_x - x, cell_y - y)
                    if dist_<min_dist:
                        min_dist = dist_
                        closest_ix, closest_iy = cell_x, cell_y
        # gradient
        if closest_ix is None:
            return (0.0,0.0)

        dx = x - closest_ix
        dy = y - closest_iy
        dist_min = math.hypot(dx, dy) + 1e-9

        # 거리越가깝 → gradient越커
        grad_scale = 1.0/(dist_min*dist_min)
        return (dx*grad_scale, dy*grad_scale)

    def normalize_angle(self, ang):
        while ang> math.pi:
            ang -= 2*math.pi
        while ang<= -math.pi:
            ang += 2*math.pi
        return ang
