from pathlib import Path
import pytest
from bootstrap_backends import verify_sources
from bootstrap_backends import source_tree
import hashlib,subprocess


def test_build_rejects_changed_source_or_patch_identity(tmp_path):
    # Identity check must happen before building/reusing a binary.
    with pytest.raises(ValueError,match='locked'):
        verify_sources(tmp_path,{'repositories':{'missing':{'ref':'0'*40,'tree_sha256':'0'*64,'patches':[]}}},['missing'])


def test_new_compiled_header_and_patch_tampering_cannot_reuse_build(tmp_path):
    source=tmp_path/'.deps/core';source.mkdir(parents=True)
    subprocess.run(['git','init','-q',str(source)],check=True)
    code=source/'core.cpp';code.write_text('int main() {}\n')
    subprocess.run(['git','-C',str(source),'add','core.cpp'],check=True)
    subprocess.run(['git','-C',str(source),'-c','user.name=test','-c','user.email=test@example.invalid','commit','-qm','fixture'],check=True)
    patch=tmp_path/'port.patch';patch.write_text('locked patch')
    spec={'ref':subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip(),
          'tree_sha256':source_tree(source),'patches':[{'path':'port.patch','sha256':hashlib.sha256(patch.read_bytes()).hexdigest()}]}
    lock={'repositories':{'core':spec}}
    verify_sources(tmp_path,lock,['core'])
    header=source/'new_compile_input.h';header.write_text('changed compile input')
    with pytest.raises(ValueError,match='source identity'):verify_sources(tmp_path,lock,['core'])
    header.unlink();patch.write_text('tampered patch')
    with pytest.raises(ValueError,match='patch identity'):verify_sources(tmp_path,lock,['core'])
