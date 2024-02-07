docker run -d -p 5000:5000 --name registry --restart=always registry:2
docker tag ros2_panoptes_encoder:latest localhost:5000/ros2_panoptes_encoder:latest
docker push localhost:5000/ros2_panoptes_encoder:latest

docker tag cam_driver_dnv:latest localhost:5000/cam_driver_dnv:latest
docker push localhost:5000/cam_driver_dnv:latest

docker image pull 10.3.0.3:5000/ros2_panoptes_encoder:latest
docker image pull 10.3.0.3:5000/cam_driver_dnv:latest
