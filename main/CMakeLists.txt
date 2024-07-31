#set(COMPONENT_SRCS "mitch_i2c.c" "switch_haptics.c" "interval.c" "core_bt_ds4.c" "switch_analog.c" "switch_spi.c" "switch_commands.c" "core_bt_switch.c" "main.c")
#set(COMPONENT_ADD_INCLUDEDIRS "")

#register_component()

idf_component_register(SRCS "main.c" "mitch_i2c.c" "core_bt_ds4.c" "core_bt_xinput.c" 
                            "core_bt_switch.c" "esp_hid_gap.c" 
                            "util_bt_hid.c" "rsc_descriptors.c" 
                            "switch_analog.c"
                            "switch_commands.c"
                            "switch_spi.c"
                            "switch_haptics.c"
                            "imu_tool.c"
                            "interval.c"
                    INCLUDE_DIRS "." "include" "include/switch")
