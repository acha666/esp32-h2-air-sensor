idf_component_register(SRCS "adafruit_lps2x.cpp" "adafruit_lps22.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer adafruit_sensor adafruit_i2c_register)
