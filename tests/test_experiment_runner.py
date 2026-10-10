"""An acceptance run starts exactly the requested owned pair and preserves qualification intent."""
import accept_continuous


def test_acceptance_start_preserves_explicit_pair_and_sensor_scene():
    assert hasattr(accept_continuous,'supervisor_arguments'),'owned pair acceptance launch missing'
    args=accept_continuous.supervisor_arguments('navigation',localization='fast_livo2',planning='ego',
        sensors='livox',scene='circle-eight',qualification=True,web=True)
    for key,value in [('--localization-backend','fast_livo2'),('--planner-backend','ego'),
                      ('--sensor-profile','livox'),('--scene','circle-eight')]:
        assert args[args.index(key)+1]==value
    assert args.count('--qualification')==1 and '--continuous' in args and '--headless' in args


def test_legacy_acceptance_does_not_silently_switch_or_qualify_a_backend():
    assert hasattr(accept_continuous,'supervisor_arguments'),'owned pair acceptance launch missing'
    args=accept_continuous.supervisor_arguments('navigation')
    assert '--qualification' not in args and '--localization-backend' not in args


def test_acceptance_archives_explicit_display_rendering_choice():
    args=accept_continuous.supervisor_arguments('navigation',rendering='mesa-display')
    assert args[args.index('--rendering')+1]=='mesa-display'


def test_native_matrix_preserves_explicit_renderer_for_every_sensor_case():
    import accept_workbench
    assert hasattr(accept_workbench,'matrix_cases')
    cases=accept_workbench.matrix_cases('mesa-display')
    assert len(cases)==12
    for name,arguments in cases:
        assert arguments[arguments.index('--rendering')+1]=='mesa-display',name
