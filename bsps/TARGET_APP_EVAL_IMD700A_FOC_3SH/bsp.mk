################################################################################
# \file bsp.mk
#
# \brief
# Define the EVAL_IMD700A_FOC_3SH target.
#
################################################################################
# \copyright
# Copyright 2020-2022 Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

# Any additional components to apply when using this board.
BSP_COMPONENTS:=

# Any additional defines to apply when using this board.
BSP_DEFINES:=

################################################################################
# ALL ITEMS BELOW THIS POINT ARE AUTO GENERATED BY THE BSP ASSISTANT TOOL.
# DO NOT MODIFY DIRECTLY. CHANGES SHOULD BE MADE THROUGH THE BSP ASSISTANT.
################################################################################

# Board device selection. MPN_LIST tracks what was selected in the BSP Assistant
# All other variables are derived by BSP Assistant based on the MPN_LIST.
MPN_LIST:=XMC1404-Q064x0128
DEVICE:=XMC1404-Q064x0128
DEVICE_COMPONENTS:=CAT3 XMC14XX
DEVICE_LIST:=XMC1404-Q064x0128
DEVICE_TOOL_IDS:=bsp-assistant device-configurator library-manager memory-analyzer online-simulator project-creator
DEVICE_XMC1404-Q064x0128_CORES:=CORE_NAME_CM0_0
DEVICE_XMC1404-Q064x0128_DIE:=XMC1400
DEVICE_XMC1404-Q064x0128_FLASH_KB:=128
DEVICE_XMC1404-Q064x0128_SRAM_KB:=16
RECIPE_DIR:=$(SEARCH_recipe-make-cat3)
