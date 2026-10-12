def test_static_gate_reports_unused_and_accepts_explicit_exports(tmp_path):
    from check_static import findings
    source=tmp_path/'module.py'
    source.write_text('import os\n')
    assert findings([source])
    source.write_text('from os import path\n__all__ = ["path"]\n')
    assert findings([source])==[]
