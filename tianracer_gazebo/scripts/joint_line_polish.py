"""Optional stationary constrained racing-line polish; no guard is relaxed.

The quadratic approximation proposes smooth normal offsets. The original
nonlinear clearance, turn, checkpoint-order and full speed-envelope checks
decide acceptance; solver success alone never makes a motion plan.
"""
import hashlib
import math
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


def polish_line(planner, seed, anchors, budgets, strength=.5):
    if isinstance(strength, bool) or not math.isfinite(strength) or not .1 <= strength <= 1.:
        raise ValueError('route_polish_strength must be between .1 and 1.0')
    anchors = np.asarray(anchors, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 2 or not np.isfinite(anchors).all() or len(anchors) < 3:
        raise ValueError('Invalid polish anchors')
    args = tuple(budgets[k] for k in ('max_speed', 'lateral_accel', 'acceleration',
                                    'braking', 'steering_rate', 'yaw_rate',
                                    'response_delay', 'speed_planning'))
    baseline = planner.score(seed, *args)
    report = dict(enabled=True, accepted=False, strength=strength,
                  baseline_model_lap_seconds=baseline, selected_model_lap_seconds=baseline,
                  route_margin=planner.margin, launch_region_protected_m=6.,
                  maximum_proposed_normal_offset=.18, knot_step=.6,
                  curvature_variation_weight=.1, solver_success=False,
                  official_result=False, model_is_not_actuator_or_tire_validation=True)
    p = seed.points
    length = float(np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1).sum())
    ds = length / len(p)
    tangent = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1)[:, None], 1e-12)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    arc = np.arange(len(p)) * ds
    launch_anchor = anchors[int(np.argmin(np.linalg.norm(anchors, axis=1)))]
    launch_s = seed.project(*launch_anchor)[0]
    from_launch = (arc - launch_s) % length
    weight = np.clip((from_launch - 6.) / 1., 0, 1) * np.clip((length - from_launch - 1.) / .8, 0, 1)
    weight = weight ** 3 * (10 - 15 * weight + 6 * weight ** 2)
    # Connected displacement interval around the existing route. These sampled
    # bounds only guide the solver; feasible() still checks the full candidate.
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
    window = max(1, round(.12 / ds))
    displacement = normal[:, :, None] * basis[:, None, :]
    second = (np.roll(displacement, window, axis=0) - 2 * displacement +
              np.roll(displacement, -window, axis=0)) / (window * ds) ** 2
    residual = (np.roll(p, window, axis=0) - 2 * p + np.roll(p, -window, axis=0)) / (window * ds) ** 2
    third = (np.roll(second, -window, axis=0) - np.roll(second, window, axis=0)) / (2 * window * ds)
    residual_rate = (np.roll(residual, -window, axis=0) - np.roll(residual, window, axis=0)) / (2 * window * ds)
    hessian = (np.einsum('nik,nil->kl', second, second) +
               .1 * np.einsum('nik,nil->kl', third, third)) / len(p) + np.eye(len(knots)) * 1e-4
    linear = (np.einsum('nik,ni->k', second, residual) +
              .1 * np.einsum('nik,ni->k', third, residual_rate)) / len(p)
    solution = minimize(lambda z: float(z @ hessian @ z + 2 * linear @ z), np.zeros(len(knots)),
                        jac=lambda z: 2 * (hessian @ z + linear), method='SLSQP',
                        bounds=Bounds(-.18, .18),
                        constraints=[LinearConstraint(basis, extents[0], extents[1])],
                        options=dict(maxiter=70, ftol=1e-9))
    report.update(solver_success=bool(solution.success), solver_message=str(solution.message),
                  solver_iterations=int(solution.nit))
    if not solution.success or not np.isfinite(solution.x).all():
        return seed, report
    best, best_score = seed, baseline
    candidates = []
    for scale in (strength, .6 * strength, .3 * strength):
        offset = basis @ solution.x * scale
        points = p + offset[:, None] * normal
        # Preserve the original guard implementation, including .34m clearance,
        # steering geometry, ordered anchors and no checkpoint bypass.
        trial = planner.feasible(points, anchors)
        if trial is None:
            candidates.append(dict(scale=scale, feasible=False))
            continue
        score = planner.score(trial, *args)
        candidates.append(dict(scale=scale, feasible=True, model_lap_seconds=score))
        if score < best_score - .002:
            best, best_score = trial, score
            report.update(accepted=True, selected_scale=scale,
                          selected_model_lap_seconds=score,
                          maximum_normal_offset=float(np.max(np.abs(offset))),
                          protected_input_displacement=float(np.max(np.abs(offset[weight == 0]))))
    report['candidates'] = candidates
    report['model_five_seconds_saved_without_startup'] = 5 * (baseline - best_score)
    report['selected_route_sha256'] = hashlib.sha256(np.round(best.points, 4).tobytes()).hexdigest()
    return best, report
