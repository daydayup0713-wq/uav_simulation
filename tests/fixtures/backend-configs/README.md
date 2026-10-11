# Pinned parameter templates for tests

These small YAML files are copied from the private dependency sources after
the patches in `dependencies/backends.lock.json`. `provenance.json` records
the exact source refs, patch hashes and file hashes. Original license files
are retained alongside the templates.

They let config contract tests execute on a clean policy checkout without
downloading or building algorithm cores. Native tests prefer the actual
installed templates. Production configuration still requires the pinned
dependency checkout; these fixtures confer no installation, replay or
flight qualification. No algorithm binary or recorded sensor data is here.
