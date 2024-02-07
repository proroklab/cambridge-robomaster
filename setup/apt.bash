apt update && apt depends nvidia-jetpack | awk '{print $2}' | uniq | xargs -I {} bash -c "apt -o Dpkg::Options::="--force-confold" -y install {} ; apt clean"
apt-get -y install vim cmake tmux

