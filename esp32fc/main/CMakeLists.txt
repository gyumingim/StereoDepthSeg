idf_component_register(
    SRCS "main.c" "ctrl_task.c" "pos_task.c" "link_task.c" "supervisor_task.c" "motor_out.c" "imu_icm42688.c" "board.c"
    INCLUDE_DIRS "."
    REQUIRES fc_safety fc_core viop driver esp_adc esp_timer esp_wifi esp_netif esp_event nvs_flash
             espressif__esp_tinyusb)
# hot loop code: optimise for speed (also set globally in sdkconfig.defaults)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wall -Wextra -Wno-unused-parameter)
