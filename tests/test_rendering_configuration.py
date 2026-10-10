import pytest


def test_display_rendering_is_explicit_and_does_not_use_broken_headless_egl():
    import importlib.util
    assert importlib.util.find_spec('rendering_configuration')
    from rendering_configuration import configure
    env={'DISPLAY':':1'}
    mode=configure('mesa-display',True,True,env)
    assert mode['headless_egl'] is False
    assert mode['environment']['__GLX_VENDOR_LIBRARY_NAME']=='mesa'
    assert mode['environment']['__EGL_VENDOR_LIBRARY_FILENAMES'].endswith('50_mesa.json')
    assert env=={'DISPLAY':':1'}
    assert configure('auto',True,True,env)['headless_egl'] is True
    assert configure('auto',False,True,env)['headless_egl'] is False
    with pytest.raises(ValueError):configure('mesa-display',True,True,{'DISPLAY':''})
    with pytest.raises(ValueError):configure('injected-driver',True,True,env)
