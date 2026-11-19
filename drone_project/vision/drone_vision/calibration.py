"""Camera geometry validation independent of ROS transport."""
import math


def calibration_error(image, info, optical_frame):
    if image.header.frame_id != optical_frame or info.header.frame_id != optical_frame:
        return 'Camera image and calibration must use the configured optical frame'
    if (image.width, image.height) != (info.width, info.height) or min(image.width, image.height) <= 0:
        return 'Calibration resolution differs from image'
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or any((
        info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height,
    )):
        return 'Use full-frame calibration at the configured output resolution'
    if not all(math.isfinite(value) for values in (info.k, info.d, info.r, info.p) for value in values):
        return 'Non-finite calibration'
    if min(info.k[0], info.k[4], info.p[0], info.p[5]) <= 0 or info.k[8] != 1 or info.p[10] != 1:
        return 'Missing or invalid intrinsics'
    if any(info.p[index] != 0 for index in (1, 3, 4, 7, 8, 9, 11)):
        return 'Use a monocular camera projection'
    identity = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
    if any(not math.isclose(actual, expected, abs_tol=1e-6) for actual, expected in zip(info.r, identity)):
        return 'Rectification must preserve the monocular optical axes'
    model_sizes = {'plumb_bob': (5,), 'rational_polynomial': (8,), 'equidistant': (4,)}
    if len(info.d) not in model_sizes.get(info.distortion_model, ()):
        return 'Unsupported distortion model or coefficient count'
    return ''


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
