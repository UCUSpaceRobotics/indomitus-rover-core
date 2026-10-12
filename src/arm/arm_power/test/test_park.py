"""Park targets and the release gate (arm_teleop/park.py, no ROS)."""
from pytest import approx

from arm_power.park import max_deviation, park_approach, park_targets, release_blockers

PARK = {'base': 0.0, 'shoulder': -0.6, 'elbow': 1.5, 'wrist': 0.2}
LIFTING = {'shoulder': +1.0, 'elbow': -1.0}
LOWERING = {'shoulder': -1.0, 'elbow': +1.0}


def test_approach_lifts_only_the_lifting_joints_a_little():
    approach = park_approach(PARK, LIFTING, 0.15)
    assert approach['shoulder'] == approx(-0.45)
    assert approach['elbow'] == approx(1.35)
    assert approach['base'] == 0.0 and approach['wrist'] == 0.2


def test_targets_press_the_lowering_joints_into_the_support():
    targets = park_targets(PARK, LOWERING, 0.03)
    assert targets['shoulder'] == approx(-0.63)
    assert targets['elbow'] == approx(1.53)
    assert targets['base'] == 0.0 and targets['wrist'] == 0.2


def test_max_deviation_names_the_worst_joint_and_counts_missing_ones():
    assert max_deviation({**PARK, 'elbow': 1.4}, PARK)[0] == 'elbow'
    current = dict(PARK)
    del current['wrist']
    assert max_deviation(current, PARK) == ('wrist', float('inf'))


def test_release_allowed_at_rest_in_the_park_pose():
    pressed = park_targets(PARK, LOWERING, 0.03)   # arm stopped by the support short of this
    near = {n: (PARK[n] + pressed[n]) / 2 for n in PARK}
    assert release_blockers(near, {n: 0.02 for n in PARK}, PARK, 0.08, 0.15) == []


def test_release_refused_away_from_park():
    reasons = release_blockers({**PARK, 'shoulder': 0.2}, {}, PARK, 0.08, 0.15)
    assert len(reasons) == 1 and 'shoulder' in reasons[0]


def test_release_refused_while_moving_but_other_joints_ignored():
    reasons = release_blockers(PARK, {'elbow': 0.4, 'gripper': 3.0}, PARK, 0.08, 0.15)
    assert len(reasons) == 1 and 'elbow' in reasons[0] and 'gripper' not in reasons[0]
