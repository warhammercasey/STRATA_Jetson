from radariq import RadarIQ

# Initialize the RadarIQ sensor (replace 'COMx' with the appropriate COM port)
sensor = RadarIQ(port='COMx', output_format=RadarIQ.OUTPUT_LIST)

# Start capturing data (continuous mode)
sensor.start(samples=0)

try:
    # Continuously fetch and print point cloud data
    for frame in sensor.get_data():
        if frame:
            print(frame)

except KeyboardInterrupt:
    # Stop capturing data on Ctrl+C
    sensor.stop()

# Close the sensor connection
sensor.close()
