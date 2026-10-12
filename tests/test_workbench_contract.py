import importlib,importlib.util
import pytest


def api():
    assert importlib.util.find_spec('workbench_contract'),'Web control and replay isolation policy missing'
    return importlib.import_module('workbench_contract')


def state(**kw):return {'armed':'False','landed':'True','offboard':'True','phase':'READY','received_at':10.,**kw}


def test_ground_selection_and_replay_never_allow_airborne_or_stale_control():
    with pytest.raises(ValueError,match='ground'):api().authorize('select',state(armed='True',landed='False'),10.1,'live')
    with pytest.raises(ValueError,match='fresh'):api().authorize('arm',state(),12.,'live')
    with pytest.raises(ValueError,match='replay'):api().authorize('land',state(armed='True',landed='False'),10.1,'replay')
    assert api().authorize('select',state(),10.1,'live')


def test_motion_requires_explicit_arm_and_hover_and_hold_land_can_interrupt():
    with pytest.raises(ValueError,match='armed'):api().authorize('takeoff',state(),10.1,'live')
    with pytest.raises(ValueError,match='hover'):api().authorize('goto',state(armed='True',landed='False',phase='MOVING'),10.1,'live')
    assert api().authorize('land',state(armed='True',landed='False',phase='MOVING'),10.1,'live')
    assert api().authorize('hold',state(armed='True',landed='False',phase='MOVING'),10.1,'live')


def test_replay_identifiers_and_scene_configuration_cannot_escape_owned_roots():
    with pytest.raises(ValueError,match='identifier'):api().recording_path('/repo','../../etc')
    with pytest.raises(ValueError,match='scene'):api().configuration({'localization':'glim','planning':'ego','scene':'arbitrary'})
    result=api().configuration({'localization':'glim','planning':'ego','scene':'circle-eight','sensor_profile':None})
    assert result['scene']=='circle-eight' and not result.get('qualification')


def test_missing_armed_flag_is_unknown_and_cannot_authorize_ground_operations():
    incomplete=state();incomplete.pop('armed')
    with pytest.raises(ValueError,match='unknown'):api().authorize('arm',incomplete,10.1,'live')


def test_rendering_choice_is_explicit_and_validated():
    from workbench_contract import configuration
    selected={'localization':'glim','planning':'fast_planner','scene':'circle-eight','sensor_profile':None,'rendering':'mesa-display'}
    assert configuration(selected)==selected
    selected['rendering']='arbitrary-shell'
    with pytest.raises(ValueError):configuration(selected)


def test_fresh_landed_failsafe_allows_owned_shutdown_but_never_rearms():
    failed=state(phase='FAILSAFE')
    assert api().authorize('stop',failed,10.1,'live')
    for command in ('select','arm','takeoff'):
        with pytest.raises(ValueError):api().authorize(command,failed,10.1,'live')
    for unsafe in (state(armed='True',landed='False',phase='FAILSAFE'),
                   state(phase='WARMUP'),state(phase='ARMING')):
        with pytest.raises(ValueError):api().authorize('stop',unsafe,10.1,'live')
    with pytest.raises(ValueError,match='fresh'):api().authorize('stop',failed,12.,'live')


def test_takeoff_submission_defers_transient_landed_flag_to_bridge_origin_guard():
    # PX4 changes landed after Offboard arming at the ground hold setpoint.
    # The bridge owns the ground-arm origin and its 0.3m takeoff constraint.
    armed=state(armed='True',landed='False',phase='HOLDING')
    assert api().authorize('takeoff',armed,10.1,'live')
    for unsafe in (dict(armed,armed='False'),dict(armed,offboard='False'),
                   dict(armed,phase='MOVING'),dict(armed,phase='FAILSAFE')):
        with pytest.raises(ValueError):api().authorize('takeoff',unsafe,10.1,'live')


def test_owned_recording_mount_is_canonicalized_but_child_escape_rejected(tmp_path):
    from workbench_contract import recording_path
    root=tmp_path/'project';root.mkdir();data=tmp_path/'owned-recordings';data.mkdir()
    (root/'recordings').symlink_to(data,target_is_directory=True)
    run=data/'valid';run.mkdir()
    assert recording_path(root,'valid')==run
    outside=tmp_path/'external';outside.mkdir()
    (data/'escape').symlink_to(outside,target_is_directory=True)
    with pytest.raises(ValueError,match='escapes'):recording_path(root,'escape')


@pytest.mark.parametrize('flag',['navigation_ready','external_ready'])
def test_reported_control_gate_failure_blocks_arm_but_allows_safe_ground_stop(flag):
    current=state(**{flag:'False'})
    with pytest.raises(ValueError,match='control health'):api().authorize('arm',current,10.1,'live')
    assert api().authorize('stop',current,10.1,'live')
