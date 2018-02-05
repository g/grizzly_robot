GRIZZLY_MAG_CONFIG=$(catkin_find --etc --first-only grizzly_base mag_config.yaml 2>/dev/null)
if [ -z "$GRIZZLY_MAG_CONFIG" ]; then
  GRIZZLY_MAG_CONFIG=$(catkin_find --share --first-only grizzly_base config/mag_config_default.yaml 2>/dev/null)
fi

export GRIZZLY_MAG_CONFIG
