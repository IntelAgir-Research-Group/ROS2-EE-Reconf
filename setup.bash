if [[ "$OSTYPE" == "darwin"* ]]; then
    SHELL_RC="$HOME/.zshrc"
else
    SHELL_RC="$HOME/.bashrc"
fi

if [ -z "$RL4GreenROS_PATH" ]; then
    export RL4GreenROS_PATH=`pwd`

    if ! grep -q "export RL4GreenROS_PATH=" "$SHELL_RC"; then
        {
            echo ""
            echo "# RL4GreenROS_PATH set on $(date)"
            echo "export RL4GreenROS_PATH=\"$user_path\""
        } >> "$SHELL_RC"
        echo "RL4GreenROS_PATH saved to $SHELL_RC"
    fi
fi

SETUP_FILE="$RL4GreenROS_PATH/workspace/install/setup.bash"
SETUP_REALSENSE_FILE="$RL4GreenROS_PATH/realsense_gazebo/install/setup.bash"
if [ -f "$SETUP_FILE" ]; then
    source "$SETUP_FILE"
    source "$SETUP_REALSENSE_FILE"
    echo "Sourced all the packages."
else
    echo "Error: setup.bash not found at $SETUP_FILE"
fi

xhost +
