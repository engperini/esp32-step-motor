idf_component_register(SRCS "main.c" "index_html.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_gpio esp_driver_ledc esp_http_server esp_wifi esp_event esp_netif nvs_flash esp_timer app_update esp_app_format esp_partition)
