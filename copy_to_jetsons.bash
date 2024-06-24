declare -a arr=(2 3)
for i in "${arr[@]}"
do
  rsync -a --progress --exclude paper --exclude docs . nvidia@10.3.1.$i:/home/nvidia/Robot
done

