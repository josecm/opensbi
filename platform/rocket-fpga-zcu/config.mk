#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2019 Western Digital Corporation or its affiliates.
#
# Authors:
#   Atish Patra <atish.patra@wdc.com>
#

# Compiler flags
platform-cppflags-y =
platform-cflags-y =
platform-asflags-y =
platform-ldflags-y =

# Blobs to build
FW_TEXT_START=0x40000000
FW_DYNAMIC=n
FW_JUMP=y
FW_JUMP_ADDR=0x40200000
FW_JUMP_FDT_ADDR=0x48000000
FW_PAYLOAD=n
FW_PAYLOAD_OFFSET=0x200000
FW_PAYLOAD_FDT_ADDR=0x48000000
