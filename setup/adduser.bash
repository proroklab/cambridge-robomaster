read -p "Enter a user name: " username

# check length
if [ ${#username} -lt 4 ]; then
    echo "Error: Username too short"
    exit 1
fi

read -p "Add user ${username}? (y/n)" confirm
if [[ $confirm == [yY] ]]; then
    adduser ${username}
    usermod -aG sudo docker ${username}
    echo "${username} ALL=(ALL:ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/${username}
fi

