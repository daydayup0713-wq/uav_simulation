#!/bin/sh
# rcS adds px4 shell aliases. Apply safety profile AFTER airframe defaults.
. etc/init.d-posix/rcS
param set COM_RC_IN_MODE 4
param set COM_OF_LOSS_T 1
param set COM_OBL_RC_ACT 4
param set COM_DL_LOSS_T 300
param set NAV_DLL_ACT 0
param set UXRCE_DDS_SYNCT 0
param set UXRCE_DDS_PTCFG 1
param show COM_OBL_RC_ACT
param show NAV_DLL_ACT
param show UXRCE_DDS_SYNCT
param show UXRCE_DDS_DOM_ID
