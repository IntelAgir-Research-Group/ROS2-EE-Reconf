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
if [ -f "$SETUP_FILE" ]; then
    source "$SETUP_FILE"
    echo "Sourced setup.bash from $SETUP_FILE"
else
    echo "Error: setup.bash not found at $SETUP_FILE"
fi