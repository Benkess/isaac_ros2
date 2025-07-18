#!/bin/bash

FASTDDS_FILE="$HOME/.ros/fastdds.xml"
ENV_EXPORT_LINE='export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds.xml'

# Create ~/.ros directory if it doesn't exist
mkdir -p "$HOME/.ros"

# Create fastdds.xml if it doesn't exist
if [ ! -f "$FASTDDS_FILE" ]; then
    echo "Creating Fast DDS profile at $FASTDDS_FILE..."
    cat > "$FASTDDS_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
    echo "✅ fastdds.xml created."
else
    echo "✅ fastdds.xml already exists at $FASTDDS_FILE."
fi

# Suggest export command
echo ""
echo "⚠️  To use this profile, run the following in each terminal where you use ROS 2:"
echo "$ENV_EXPORT_LINE"
