# Extra script für micro-ROS PlatformIO Integration
# Wird automatisch vor dem Build ausgeführt

Import("env")

# micro-ROS Build-Konfiguration
env.Append(
    CPPDEFINES=[
        ("MICROROS_TRANSPORT", "serial"),
    ]
)
