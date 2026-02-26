#!/bin/bash

# Configuration
REMOTE="benjamin@zeromotor.local"
REMOTE_DIR="~/balance_bot_2"
LOG_FILE="remote_logs.txt"

echo "Syncing code to $REMOTE..."
rsync -avz --exclude '.git' --exclude '.venv' --exclude '__pycache__' --exclude 'remote_logs.txt' ./ $REMOTE:$REMOTE_DIR/

echo "Running Tabula Rasa Discovery on $REMOTE..."
echo "This will take a few moments. Streaming output below and saving to $LOG_FILE..."

# Run via SSH, stream to terminal, and tee to the log file so Jules can read it
ssh -t $REMOTE "cd $REMOTE_DIR && uv run balance-bot" | tee $LOG_FILE

echo ""
echo "Done! The output has been saved to $LOG_FILE so Jules can analyze it."