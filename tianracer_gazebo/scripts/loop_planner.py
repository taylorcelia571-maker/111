"""Stationary full-loop planning against an occupancy grid; no ROS dependency."""
import hashlib
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates, gaussian_filter1d
from race_line import ClosedPath, positive


class LoopPlanner:
    def __init__(self, occupancy, resolution, origin, margin=0.26):
        self.resolution = positive('resolution', resolution)
        self.margin = positive('margin', margin)
        self.origin = np.asarray(origin, dtype=float)
        grid = np.asarray(occupancy)
        if grid.ndim != 2 or not np.isfinite(self.origin).all() or self.origin.shape != (2,):
            raise ValueError('Invalid map geometry')
        # Unknown cells are obstacles. Subtract a cell diagonal for conservative interpolation.
        self.clearance = distance_transform_edt(grid == 0) * resolution - resolution * math.sqrt(2)
        self.allowed = self.clearance >= margin

    def clearance_at(self, points):
        p = np.asarray(points)
        cells = (p - self.origin) / self.resolution - 0.5
        return map_coordinates(self.clearance, [cells[:, 1], cells[:, 0]],
                               order=1, mode='constant', cval=-1)

    def segment_check(self, points):
        a = np.asarray(points)
        b = np.roll(a, -1, axis=0)
        n = max(1, int(math.ceil(np.max(np.linalg.norm(b-a, axis=1)) / (self.resolution / 2))))
        return min(float(self.clearance_at(a + (b-a)*t).min()) for t in np.linspace(0, 1, n+1))

    def edge(self, start, goal, clearance_weight=8.0):
        def cell(p):
            c = np.floor((np.asarray(p)-self.origin)/self.resolution).astype(int)
            if not (0 <= c[0] < self.allowed.shape[1] and 0 <= c[1] < self.allowed.shape[0]):
                raise ValueError('Waypoint outside map')
            q = (int(c[1]), int(c[0]))
            if not self.allowed[q]: raise ValueError('Waypoint lacks vehicle clearance')
            return q
        start, goal = cell(start), cell(goal)
        distances = {start: 0.0}
        parent = {}
        queue = [(math.hypot(start[0]-goal[0], start[1]-goal[1]), 0.0, start)]
        moves = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        h, w = self.allowed.shape
        while queue:
            _, cost, a = heapq.heappop(queue)
            if cost != distances.get(a): continue
            if a == goal:
                result = [a]
                while result[-1] != start: result.append(parent[result[-1]])
                return (np.array(result[::-1])[:, ::-1] + 0.5)*self.resolution + self.origin
            for dy, dx, length in moves:
                b = a[0]+dy, a[1]+dx
                if not (0 <= b[0] < h and 0 <= b[1] < w) or not self.allowed[b]: continue
                if dy and dx and (not self.allowed[a[0]+dy, a[1]] or not self.allowed[a[0], a[1]+dx]): continue
                penalty = 1 + clearance_weight*math.exp(-max(0, self.clearance[b]-self.margin)/0.18)
                new = cost + length*penalty
                if new < distances.get(b, float('inf')):
                    distances[b], parent[b] = new, a
                    heapq.heappush(queue, (new+math.hypot(b[0]-goal[0], b[1]-goal[1]), new, b))
        raise ValueError('No connected collision-free route between waypoints')

    def feasible(self, points, anchors, max_steer=0.48, wheelbase=0.26):
        if self.segment_check(points) < self.margin: return None
        trial=ClosedPath(np.vstack([points,points[0]]),spacing=0.04)
        if self.segment_check(trial.points) < self.margin: return None
        d=np.roll(trial.points,-1,axis=0)-trial.points
        yaw=np.arctan2(d[:,1],d[:,0])
        turns=np.arctan2(np.sin(np.roll(yaw,-1)-yaw),np.cos(np.roll(yaw,-1)-yaw))/trial.ds
        if np.max(np.abs(turns)) > math.tan(max_steer)/wheelbase*.95: return None
        projected=[trial.project(*p) for p in anchors]
        if any(error>.4 for _,error in projected): return None
        positions=[distance for distance,_ in projected]
        increments=np.diff(np.r_[positions,positions[0]+trial.length])%trial.length
        if not math.isclose(float(increments.sum()),trial.length,rel_tol=1e-6): return None
        return trial

    @staticmethod
    def estimate(trial, max_speed, lateral_accel, acceleration, braking, steering_rate, yaw_rate, response_delay=0.0):
        from predictive_follower import PredictiveFollower
        tangent=np.roll(trial.points,-1,axis=0)-np.roll(trial.points,1,axis=0)
        tangent/=np.linalg.norm(tangent,axis=1)[:,None]
        rear=trial.points-.13*tangent
        reference=ClosedPath(np.vstack([rear,rear[0]]),spacing=.04)
        follower=PredictiveFollower(reference,max_speed=max_speed,lateral_accel=lateral_accel,
                           acceleration=acceleration,braking=braking,steering_rate=steering_rate,yaw_rate=yaw_rate,
                           response_delay=response_delay)
        return float(np.sum(reference.ds/follower.preview_speed_profile()))

    def build(self, waypoints, max_steer=0.48, wheelbase=0.26, optimize_time=False,
              max_speed=4.0, lateral_accel=5.0, acceleration=2.0, braking=3.0,
              clearance_weight=None, steering_rate=2.0, yaw_rate=3.2, refine_line=False, response_delay=0.0):
        if not math.isfinite(response_delay) or not 0 <= response_delay <= .5:
            raise ValueError("response_delay must be between 0 and 0.5 seconds")
        if optimize_time:
            for name,value in (('max_speed',max_speed),('lateral_accel',lateral_accel),
                               ('acceleration',acceleration),('braking',braking),('steering_rate',steering_rate),('yaw_rate',yaw_rate)):
                positive(name,value)
        if optimize_time and clearance_weight is None:
            best = None
            best_selection = None
            for weight in (2.0,4.0,8.0,16.0):
                try:
                    trial = self.build(waypoints,max_steer,wheelbase,True,max_speed,
                                       lateral_accel,acceleration,braking,weight,steering_rate,yaw_rate,
                                       response_delay=response_delay)
                except ValueError:
                    continue  # infeasible alternatives never become motion plans
                selection = dict(self.selection,clearance_weight=weight)
                if best_selection is None or selection['estimated_lap_seconds'] < best_selection['estimated_lap_seconds']:
                    best,best_selection = trial,selection
            if best is None: raise ValueError('No time-optimized route meets geometry/clearance constraints')
            # Explore modest inward/outward corner offsets in the same corridor.
            # No new topology or checkpoint bypass is introduced.
            seed=best
            tangent=np.roll(seed.points,-1,axis=0)-np.roll(seed.points,1,axis=0)
            tangent/=np.linalg.norm(tangent,axis=1)[:,None]
            normal=np.column_stack([-tangent[:,1],tangent[:,0]])
            shape=gaussian_filter1d(np.tanh(seed.curvature/.5),.35/seed.ds,mode='wrap')
            best_selection['corner_offset']=0.0
            for offset in (-.12,-.08,-.04,.04,.08,.12):
                candidate=self.feasible(seed.points+offset*shape[:,None]*normal,np.asarray(waypoints),max_steer,wheelbase)
                if candidate is None: continue
                estimated=self.estimate(candidate,max_speed,lateral_accel,acceleration,braking,steering_rate,yaw_rate,response_delay)
                if estimated < best_selection['estimated_lap_seconds']:
                    best=candidate
                    best_selection.update(estimated_lap_seconds=estimated,corner_offset=offset)
            self.baseline_points=best.points.copy()
            baseline_time=best_selection['estimated_lap_seconds']
            accepted=0
            if refine_line:
                # Coordinate descent on separate corner regions, not one shared
                # offset for all bends. Geometry is checked for every trial.
                peaks=[]
                for index in np.argsort(-np.abs(best.curvature)):
                    distance=index*best.ds
                    if abs(best.curvature[index])<.35: break
                    if all(min(abs(distance-p),best.length-abs(distance-p))>1.2 for p in peaks):
                        peaks.append(distance)
                    if len(peaks)>=12: break
                seed_length=best.length
                for _ in range(2):
                    for peak in peaks:
                        current=best
                        tangent=np.roll(current.points,-1,axis=0)-np.roll(current.points,1,axis=0)
                        tangent/=np.linalg.norm(tangent,axis=1)[:,None]
                        normal=np.column_stack([-tangent[:,1],tangent[:,0]])
                        arc=np.arange(current.n)*current.ds
                        distance=np.abs(arc-peak/seed_length*current.length)
                        distance=np.minimum(distance,current.length-distance)
                        bump=np.exp(-.5*(distance/.6)**2)
                        for offset in (-.06,-.03,.03,.06):
                            candidate=self.feasible(current.points+offset*bump[:,None]*normal,np.asarray(waypoints),max_steer,wheelbase)
                            if candidate is None: continue
                            estimated=self.estimate(candidate,max_speed,lateral_accel,acceleration,braking,steering_rate,yaw_rate,response_delay)
                            if estimated < best_selection['estimated_lap_seconds']-1e-4:
                                best=candidate
                                best_selection['estimated_lap_seconds']=estimated
                                accepted+=1
            best_selection.update(scoring_response_delay=response_delay,
                                  scoring_model='nominal_speed_response_preview_proxy',
                                  scoring_includes_startup=False,
                                  refinement_moves=accepted,baseline_model_lap_seconds=baseline_time,
                                  refinement_gain_seconds=baseline_time-best_selection['estimated_lap_seconds'],
                                  route_changed=accepted>0,
                                  route_sha256=hashlib.sha256(np.round(best.points,4).tobytes()).hexdigest())
            self.selection = best_selection
            return best
        weight = 8.0 if clearance_weight is None else positive('clearance_weight',clearance_weight)
        anchors = np.asarray(waypoints, dtype=float)
        if anchors.ndim != 2 or anchors.shape[1] != 2 or len(anchors) < 3 or not np.isfinite(anchors).all():
            raise ValueError('Invalid ordered waypoints')
        raw = np.concatenate([self.edge(a, b,weight)[:-1] for a,b in zip(anchors, np.roll(anchors,-1,axis=0))])
        path = ClosedPath(np.vstack([raw, raw[0]]), spacing=0.04)
        original = path.points.copy()
        # Periodic smoothing also smooths the finish-line seam. Each candidate must
        # retain clearance and remain within the same corridor as the A* reference.
        chosen = None
        best_time = float('inf')
        self.selection = {}
        for width in np.arange(0.12, 0.71, 0.02):
            candidate = gaussian_filter1d(original, width/path.ds, axis=0, mode='wrap')
            if np.max(np.linalg.norm(candidate-original, axis=1)) > 0.40: continue
            trial=self.feasible(candidate,anchors,max_steer,wheelbase)
            if trial is None: continue
            if optimize_time:
                estimated=self.estimate(trial,max_speed,lateral_accel,acceleration,braking,steering_rate,yaw_rate,response_delay)
                if estimated >= best_time: continue
                best_time = estimated
                self.selection = dict(estimated_lap_seconds=estimated,smoothing_width=float(width))
            chosen = trial  # legacy: prefer smoothest feasible; optimized: lowest modeled time
        if chosen is None:
            raise ValueError('Route cannot meet clearance and steering limits; do not drive')
        return chosen
