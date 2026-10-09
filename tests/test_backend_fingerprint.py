import json
from pathlib import Path
from backend_provenance import implementation_fingerprint


def test_backend_fingerprint_includes_compiled_source_port_and_adapter_but_not_other_backend(tmp_path):
    lock={'backends':{'one':{'repositories':['core']},'two':{'repositories':['other']}},
          'repositories':{'core':{'ref':'a'*40,'tree_sha256':'1'*64,'patches':[]},'other':{'ref':'b'*40}}}
    adapter=tmp_path/'adapter.py';adapter.write_text('source-time conversion v1')
    initial=implementation_fingerprint(tmp_path,lock,'one',['adapter.py'])
    lock['repositories']['other']['ref']='c'*40
    assert implementation_fingerprint(tmp_path,lock,'one',['adapter.py'])==initial
    adapter.write_text('source-time conversion v2')
    assert implementation_fingerprint(tmp_path,lock,'one',['adapter.py'])!=initial
    adapter.write_text('source-time conversion v1');lock['repositories']['core']['tree_sha256']='2'*64
    assert implementation_fingerprint(tmp_path,lock,'one',['adapter.py'])!=initial
