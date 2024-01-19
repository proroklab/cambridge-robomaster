apt update && apt depends nvidia-jetpack | awk '{print $2}' | uniq | xargs -I {} bash -c "sudo apt -o Dpkg::Options::="--force-confold" -y install {} ; sudo apt clean"
apt install vim cmake tmux

