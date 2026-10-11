"""Explicit per-run rendering environment; never modifies host drivers/settings."""


def configure(mode,headless,sensors,environment):
    if mode not in ('auto','mesa-display'):raise ValueError('unknown rendering mode')
    env=dict(environment)
    if mode=='mesa-display' and sensors:
        if not env.get('DISPLAY'):raise ValueError('mesa-display requires an existing DISPLAY session')
        env.update(__GLX_VENDOR_LIBRARY_NAME='mesa',
            __EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/50_mesa.json')
    return {'mode':mode,'environment':env,'headless_egl':bool(headless and sensors and mode=='auto'),
        'scope':'per-run rendering selection; no GPU accuracy/performance or flight qualification inferred'}
