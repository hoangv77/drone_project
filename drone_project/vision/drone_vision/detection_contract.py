"""Validation rules for offline tracker observations."""

def valid_observation(stamp, now, max_age, frame, optical, calibrated, last_stamp, publishers):
    return (stamp > 0 and 0 <= now - stamp <= max_age and stamp > last_stamp
            and frame == optical and stamp in calibrated
            and publishers == [('/', 'apriltag')])


def valid_targets(detections, allowed_ids, family):
    ids = [d.id for d in detections]
    return (len(ids) == len(set(ids)) and all(
        d.id in allowed_ids and d.family.removeprefix('tag') == family
        for d in detections))
