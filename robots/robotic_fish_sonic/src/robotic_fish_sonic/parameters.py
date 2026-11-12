"""Canonical private parameter names with legacy launch/YAML compatibility."""
import rospy


def get_compatible_param(name, default, *legacy_names):
    """Read the first explicitly set name; canonical names take precedence.

    Presence, rather than truthiness, matters: false, zero and an empty string
    are legitimate explicit settings. Legacy names are read, never republished.
    """
    for candidate in (name,) + legacy_names:
        if rospy.has_param(candidate):
            return rospy.get_param(candidate)
    return default
