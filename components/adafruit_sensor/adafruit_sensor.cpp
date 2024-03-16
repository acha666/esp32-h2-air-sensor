#include "esp_log.h"
#include "Adafruit_Sensor.h"

static const char *TAG = "SensorDetails";

void Adafruit_Sensor::printSensorDetails(void)
{
    sensor_t sensor;
    getSensor(&sensor);
    ESP_LOGI(TAG, "------------------------------------");
    ESP_LOGI(TAG, "Sensor:       %s", sensor.name);
    ESP_LOGI(TAG, "Type:         ");
    switch ((sensors_type_t)sensor.type)
    {
    case SENSOR_TYPE_ACCELEROMETER:
        ESP_LOGI(TAG, "Acceleration (m/s2)");
        break;
    case SENSOR_TYPE_MAGNETIC_FIELD:
        ESP_LOGI(TAG, "Magnetic (uT)");
        break;
    case SENSOR_TYPE_ORIENTATION:
        ESP_LOGI(TAG, "Orientation (degrees)");
        break;
    case SENSOR_TYPE_GYROSCOPE:
        ESP_LOGI(TAG, "Gyroscopic (rad/s)");
        break;
    case SENSOR_TYPE_LIGHT:
        ESP_LOGI(TAG, "Light (lux)");
        break;
    case SENSOR_TYPE_PRESSURE:
        ESP_LOGI(TAG, "Pressure (hPa)");
        break;
    case SENSOR_TYPE_PROXIMITY:
        ESP_LOGI(TAG, "Distance (cm)");
        break;
    case SENSOR_TYPE_GRAVITY:
        ESP_LOGI(TAG, "Gravity (m/s2)");
        break;
    case SENSOR_TYPE_LINEAR_ACCELERATION:
        ESP_LOGI(TAG, "Linear Acceleration (m/s2)");
        break;
    case SENSOR_TYPE_ROTATION_VECTOR:
        ESP_LOGI(TAG, "Rotation vector");
        break;
    case SENSOR_TYPE_RELATIVE_HUMIDITY:
        ESP_LOGI(TAG, "Relative Humidity (%%)");
        break;
    case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        ESP_LOGI(TAG, "Ambient Temp (C)");
        break;
    case SENSOR_TYPE_OBJECT_TEMPERATURE:
        ESP_LOGI(TAG, "Object Temp (C)");
        break;
    case SENSOR_TYPE_VOLTAGE:
        ESP_LOGI(TAG, "Voltage (V)");
        break;
    case SENSOR_TYPE_CURRENT:
        ESP_LOGI(TAG, "Current (mA)");
        break;
    case SENSOR_TYPE_COLOR:
        ESP_LOGI(TAG, "Color (RGBA)");
        break;
    case SENSOR_TYPE_TVOC:
        ESP_LOGI(TAG, "Total Volatile Organic Compounds (ppb)");
        break;
    case SENSOR_TYPE_VOC_INDEX:
        ESP_LOGI(TAG, "Volatile Organic Compounds (Index)");
        break;
    case SENSOR_TYPE_NOX_INDEX:
        ESP_LOGI(TAG, "Nitrogen Oxides (Index)");
        break;
    case SENSOR_TYPE_CO2:
        ESP_LOGI(TAG, "Carbon Dioxide (ppm)");
        break;
    case SENSOR_TYPE_ECO2:
        ESP_LOGI(TAG, "Equivalent/estimated CO2 (ppm)");
        break;
    case SENSOR_TYPE_PM10_STD:
        ESP_LOGI(TAG, "Standard Particulate Matter 1.0 (ppm)");
        break;
    case SENSOR_TYPE_PM25_STD:
        ESP_LOGI(TAG, "Standard Particulate Matter 2.5 (ppm)");
        break;
    case SENSOR_TYPE_PM100_STD:
        ESP_LOGI(TAG, "Standard Particulate Matter 10.0 (ppm)");
        break;
    case SENSOR_TYPE_PM10_ENV:
        ESP_LOGI(TAG, "Environmental Particulate Matter 1.0 (ppm)");
        break;
    case SENSOR_TYPE_PM25_ENV:
        ESP_LOGI(TAG, "Environmental Particulate Matter 2.5 (ppm)");
        break;
    case SENSOR_TYPE_PM100_ENV:
        ESP_LOGI(TAG, "Environmental Particulate Matter 10.0 (ppm)");
        break;
    case SENSOR_TYPE_GAS_RESISTANCE:
        ESP_LOGI(TAG, "Gas Resistance (ohms)");
        break;
    case SENSOR_TYPE_UNITLESS_PERCENT:
        ESP_LOGI(TAG, "Unitless Percent (%%)");
        break;
    case SENSOR_TYPE_ALTITUDE:
        ESP_LOGI(TAG, "Altitude (m)");
        break;
    default:
        ESP_LOGI(TAG, "Unknown sensor type");
        break;
    }

    ESP_LOGI(TAG, "Driver Ver:   %ld", sensor.version);
    ESP_LOGI(TAG, "Unique ID:    %ld", sensor.sensor_id);
    ESP_LOGI(TAG, "Min Value:    %f", sensor.min_value);
    ESP_LOGI(TAG, "Max Value:    %f", sensor.max_value);
    ESP_LOGI(TAG, "Resolution:   %f", sensor.resolution);
    ESP_LOGI(TAG, "------------------------------------");
}
