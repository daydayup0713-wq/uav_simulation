# Third-party interface patches

`backends.lock.json` records the exact source revision, ordered patches and
patched source-tree hash. The FAST cores are downloaded into ignored `.deps`
directories; these patches adapt installation and ROS interfaces and preserve
upstream algorithm logic.

- FAST-LIO2: hku-mars/FAST_LIO, ROS2 branch, GPL-2.0.
- FAST-LIVO2: hku-mars/FAST-LIVO2, GPL-2.0. The five patches in
  `fast_livo2-intel/` come from open-edge-platform/robotics-ai-suite at the
  locked revision. Their original author/commit information is preserved.
- FAST-LIVO2-RTK: sb-im/FAST-LIVO2-RTK-ROS2 community port of
  xuankuzcr/FAST-LIVO2-RTK, GPL-2.0. Platform patches adjust private linkage,
  sensor QoS, source-time translation, bounded transport queues and empty
  point-cloud export. The pose keyframes and optimizer mathematics are retained.
- Sophus: private old/new versions; the GCC11 compatibility patch concerns
  the old version. Its upstream MIT notice is preserved in `licenses`.
- Vikit: private build/parameter-service changes; pinned upstream package
  metadata declares GPLv3. Source files retain their original notices.

Patches to third-party files follow those files' licenses. The original FAST
GPL text and Sophus MIT notice are provided in `licenses/`. The message-only
Livox interface has its original MIT notice in
`localization/interfaces/livox_ros_driver2/LICENSE`.
