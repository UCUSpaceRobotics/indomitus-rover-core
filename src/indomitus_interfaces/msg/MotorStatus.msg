# if parameter is not available it's uqeal to Nan
uint8   esc_id         # CAN device address
string  motor_type     # "steadywin" or "damiao"
string  joint_name     # URDF joint name

# Kinematic state
float32 position       # rad (output shaft)
float32 velocity       # rad/s (output shaft)
float32 torque         # Nm
bool    kinematic_valid

# Electrical / health state
float32 voltage        # V   (Steadywin 0xAE)
float32 current        # A   (Steadywin 0xAE)
float32 temperature    # °C
uint8   mode           # 0=off 1=voltage 2=Iq 3=speed 4=position
uint8   fault_code     # bitmask: Bit0=voltage Bit1=current Bit2=temp Bit3=encoder Bit6=hw Bit7=sw
bool    health_valid

bool    enabled        # true if motor is actively controlled
