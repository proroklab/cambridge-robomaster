docker run -d -p 5000:5000 --name registry --restart=always registry:2
docker tag ros2_panoptes_encoder:latest localhost:5000/ros2_panoptes_encoder:latest
docker push localhost:5000/ros2_panoptes_encoder:latest

Configuring default runtime essential for building with cuda dependencies!
sudo vim /etc/docker/daemon.json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia",
    "insecure-registries": ["10.3.0.3:5000"]
}
sudo service docker restart
docker image pull 10.3.0.3:5000/ros2_panoptes_encoder:latest
