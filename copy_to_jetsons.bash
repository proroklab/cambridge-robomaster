for ip in 10.3.1.2 10.3.1.3 10.3.1.4; do
  rsync -a --delete . nvidia@$ip:/home/nvidia/Robot
done

