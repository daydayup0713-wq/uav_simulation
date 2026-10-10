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
