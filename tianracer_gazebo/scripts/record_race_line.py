#!/usr/bin/env python3
"""Record a dense full lap. Official validity must be checked by the user."""
import math
import os
import numpy as np
import rospy
import tf
from race_line import ClosedPath


def main():
    rospy.init_node('record_race_line')
    filename = os.path.expanduser(rospy.get_param('~output'))
    if os.path.exists(filename): raise ValueError('Output already exists; choose a new filename')
    listener = tf.TransformListener()
    frame = rospy.get_param('~map_frame', 'map')
    base = rospy.get_param('~base_frame', rospy.get_namespace().strip('/') + '/base_footprint')
    points, headings = [], []
    length = 0.0
    rate = rospy.Rate(20)
    previous = rospy.Time.now()
    rospy.loginfo('Recording localization. Drive one full lap and verify official lap count.')
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now < previous: raise ValueError('Simulation clock reset; restart recording')
        previous = now
        try:
            stamp = listener.getLatestCommonTime(frame, base)
            if not 0 <= (now - stamp).to_sec() <= 0.3: raise tf.Exception('Stale TF')
            xyz, quat = listener.lookupTransform(frame, base, rospy.Time(0))
        except tf.Exception:
            if points: raise ValueError('Localization lost while recording; discard incomplete path')
            rate.sleep()
            continue
        xy = np.array(xyz[:2])
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        if not np.isfinite(xy).all() or not math.isfinite(yaw): raise ValueError('Invalid pose')
        gap = np.linalg.norm(xy - points[-1]) if points else 0.0
        if gap > 0.5: raise ValueError('Localization jump; discard path')
        if not points or gap >= 0.05:
            points.append(xy)
            headings.append(yaw)
            length += gap
        if length > 20 and len(points) > 100:
            heading_error = math.atan2(math.sin(yaw - headings[0]), math.cos(yaw - headings[0]))
            if np.linalg.norm(xy - points[0]) < 0.25 and abs(heading_error) < 0.35:
                path = ClosedPath(points)
                os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
                with open(filename, 'x') as out:
                    np.savetxt(out, np.asarray(points), delimiter=',', fmt='%.5f',
                               header='Recorded localization route; verify one official valid lap before use')
                rospy.loginfo('Saved %.1fm path to %s. Discard if referee did not count a valid lap.',
                              path.length, filename)
                return
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        rospy.logfatal('Recording failed: %s', error)
        raise SystemExit(1)
