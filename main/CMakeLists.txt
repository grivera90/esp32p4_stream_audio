# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

file (GLOB srcs_files "../audio_capture/*.c" "../wifi_manager/*.c" "../audio_udp_streamer/*.c")
file (GLOB incs_files "../audio_capture" "../wifi_manager" "../audio_udp_streamer")

idf_component_register(
    SRCS "main.c" ${srcs_files}
    INCLUDE_DIRS ${incs_files}

    REQUIRES 	esp_driver_i2s 
    			esp_driver_gpio
    			esp_wifi
    			nvs_flash
    			esp_event
    			esp_netif
    			lwip
    			
    PRIV_REQUIRES
)
