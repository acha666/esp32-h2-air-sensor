idf_component_register(SRCS "Adafruit_LPS2X.cpp" "Adafruit_LPS22.cpp" "Adafruit_LPS25.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver adafruit_sensor i2c_register)
