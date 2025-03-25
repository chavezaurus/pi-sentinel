#!/bin/bash

## sync the time
sudo ntpdate -q 0.us.pool.ntp.org;

## activate python virtual environment
if [ ! -d "$HOME/.venv" ]
then 
    echo "Could not find venv. Creating..."
    cd ~/ && /usr/bin/python3 -m venv .venv
    ~/.venv/bin/pip install -r ~/pi-sentinel/requirements.txt
else
    echo "Found venv"
fi

tmux start-server
tmux new-session -d -s sentinel
tmux send-keys -t sentinel:0 ". ~/.venv/bin/activate && cd ~/pi-sentinel && python3 sentinel.py" ENTER

echo "To view the Pi-Sentinel interface, attach any browser to [raspberry pi ip_address]:9090"
echo "To see Pi-Sentinel messages, type in: tmux attach"
echo "To stop Pi-Sentinel, type in: tmux kill-session"
