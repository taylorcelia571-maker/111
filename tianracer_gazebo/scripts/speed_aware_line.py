"""Stationary racing-line refinement against the complete cyclic speed model.

The optimizer proposes offsets; original geometry/checkpoint checks decide
acceptance. No live limit, sensor, braking allowance or referee is modified.
"""
import hashlib
import math
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize
from race_line import ClosedPath


def normal_search_space(planner, seed, anchors):
    p = seed.points
    length = float(np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1).sum())
    ds = length / len(p)
    tangent = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-12)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    arc = np.arange(len(p)) * ds
    launch = anchors[int(np.argmin(np.linalg.norm(anchors, axis=1)))]
    relative = (arc - seed.project(*launch)[0]) % length
    weight = np.clip((relative - 6.) / 1., 0, 1) * np.clip((length - relative - 1.) / .8, 0, 1)
    weight = weight ** 3 * (10 - 15 * weight + 6 * weight ** 2)
    extents = []
    for sign in (-1., 1.):
        extent = np.zeros(len(p))
        active = np.ones(len(p), dtype=bool)
        for offset in np.arange(.003, .181, .003):
            active &= planner.clearance_at(p + sign * offset * normal) >= planner.margin + 1e-6
            extent[active] = offset
        extents.append(sign * extent)
    knots = np.arange(0., length, .6)
    distance = np.abs(arc[:, None] - knots[None, :])
    distance = np.minimum(distance, length - distance)
    basis = np.exp(-.5 * (distance / (.6 * .7)) ** 2)
    basis /= basis.sum(axis=1)[:, None]
    basis *= weight[:, None]
    return normal, basis, weight, LinearConstraint(basis, extents[0], extents[1])


def reference_peaks(center):
    tangent = np.roll(center.points, -1, axis=0) - np.roll(center.points, 1, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-12)
    rear = center.points - .13 * tangent
    path = ClosedPath(np.vstack([rear, rear[0]]), spacing=.04)
    angle = np.arctan(.26 * path.curvature)
    return dict(curvature=float(np.max(np.abs(path.curvature))),
                steering_gradient=float(np.max(np.abs(np.roll(angle, -1) - angle)) / path.ds))


def refine_speed_line(planner, seed, anchors, budgets, iterations=6, source_seed=None):
    if isinstance(iterations, bool) or not isinstance(iterations, int) or not 1 <= iterations <= 10:
        raise ValueError('route_speed_iterations must be an integer between 1 and 10')
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 2 or len(anchors) < 3 or not np.isfinite(anchors).all():
        raise ValueError('Invalid speed-refinement anchors')
    args = tuple(budgets[k] for k in ('max_speed', 'lateral_accel', 'acceleration', 'braking',
                                    'steering_rate', 'yaw_rate', 'response_delay', 'speed_planning'))
    baseline = planner.score(seed, *args)
    best = [baseline, seed]
    quality_limits = reference_peaks(seed)
    report = dict(enabled=True, accepted=False, objective='complete_cyclic_speed_envelope',
                  baseline_model_lap_seconds=baseline, selected_model_lap_seconds=baseline,
                  requested_iterations=iterations, evaluations=0, feasible_improvements=0,
                  route_margin=planner.margin, launch_region_protected_m=6.,
                  official_result=False, optimizer_convergence_is_not_geometry_validation=True,
                  initial_reference_peak_limits=dict(quality_limits), portfolio=[])
    # Restart from the unpolished Level-3 line: optimizing only the already
    # half-shifted incumbent can get trapped at a different local solution.
    if source_seed is not None:
        from joint_line_polish import polish_line
        proposal = source_seed
        for pass_id in range(2):
            proposal, stage = polish_line(planner, proposal, anchors, budgets, strength=1.)
            item = dict(pass_id=pass_id + 1, solver_accepted=stage['accepted'], accepted=False)
            if stage['accepted']:
                feasible = planner.feasible(proposal.points, anchors)
                if feasible is not None:
                    peaks = reference_peaks(feasible)
                    quality_ok = all(peaks[k] <= quality_limits[k] * 1.001 + 1e-9 for k in peaks)
                    score = planner.score(feasible, *args)
                    item.update(reference_peaks=peaks, quality_ok=quality_ok, model_lap_seconds=score)
                    if quality_ok and score < best[0] - .002:
                        best[:] = [score, feasible]
                        item['accepted'] = True
                        report.update(accepted=True, selected_model_lap_seconds=score)
            report['portfolio'].append(item)
            if not stage['accepted']:
                break
    search_seed = best[1]
    quality_limits = reference_peaks(search_seed)
    report['local_reference_peak_limits'] = dict(quality_limits)
    normal, basis, weight, constraint = normal_search_space(planner, search_seed, anchors)

    def objective(z):
        report['evaluations'] += 1
        if not np.isfinite(z).all():
            return 1e6
        offset = basis @ z
        points = search_seed.points + offset[:, None] * normal
        try:
            trial = ClosedPath(np.vstack([points, points[0]]), spacing=.04)
            score = planner.score(trial, *args)
            clearance = planner.segment_check(trial.points)
            penalty = 1e4 * max(0., planner.margin - clearance) ** 2
            direction = np.roll(trial.points, -1, axis=0) - trial.points
            yaw = np.arctan2(direction[:, 1], direction[:, 0])
            turn = np.arctan2(np.sin(np.roll(yaw, -1) - yaw),
                              np.cos(np.roll(yaw, -1) - yaw)) / trial.ds
            penalty += 10 * max(0., float(np.max(np.abs(turn))) - math.tan(.48) / .26 * .95) ** 2
            peaks = reference_peaks(trial)
            penalty += sum(200 * max(0., peaks[k] - quality_limits[k] * .995) ** 2 for k in peaks)
            # Penalties only guide the search. They NEVER replace feasible().
            if score + penalty < best[0] - .002:
                feasible = planner.feasible(points, anchors)
                if feasible is not None:
                    actual_peaks = reference_peaks(feasible)
                    if any(actual_peaks[k] > quality_limits[k] * 1.001 + 1e-9 for k in actual_peaks):
                        return score + penalty
                    exact = planner.score(feasible, *args)
                    if exact < best[0] - .002:
                        best[:] = [exact, feasible]
                        report.update(accepted=True, selected_model_lap_seconds=exact,
                                      maximum_normal_offset=float(np.max(np.abs(offset))),
                                      protected_input_displacement=float(np.max(np.abs(offset[weight == 0]))))
                        report['feasible_improvements'] += 1
            return score + penalty
        except (ValueError, FloatingPointError):
            return 1e6

    try:
        result = minimize(objective, np.zeros(basis.shape[1]), method='SLSQP',
                          bounds=Bounds(-.18, .18), constraints=[constraint],
                          options=dict(maxiter=iterations, ftol=2e-4, eps=.002))
        report.update(solver_success=bool(result.success), solver_message=str(result.message),
                      solver_iterations=int(result.nit))
    except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
        report.update(solver_success=False, solver_message=str(error))
    # Finite iterates can be retained after an iteration limit only because each
    # retained candidate independently passed original geometry and exact score.
    report.update(selected_route_sha256=hashlib.sha256(np.round(best[1].points, 4).tobytes()).hexdigest(),
                  selected_reference_peaks=reference_peaks(best[1]),
                  model_five_seconds_saved_without_startup=5 * (baseline - best[0]))
    return best[1], report


def sector_speed_report(path):
    """Planning diagnostics only. Time bins below partition reference geometry."""
    curve = np.abs(path.curvature)
    masks = dict(straight_or_gentle=curve < .15, transition=(curve >= .15) & (curve < .6), bend=curve >= .6)
    result = {}
    for name, mask in masks.items():
        speed = path.speeds[mask]
        result[name] = dict(length_m=float(mask.sum() * path.ds),
            model_lap_seconds=float(np.sum(path.ds / speed)),
            minimum_planned_speed=float(speed.min()) if len(speed) else None,
            median_planned_speed=float(np.median(speed)) if len(speed) else None,
            maximum_planned_speed=float(speed.max()) if len(speed) else None)
    return dict(model_only=True, reference_bins_are_disjoint=True, sectors=result)
