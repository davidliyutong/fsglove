# Hand API Server

This is the API server for the 16 IMU mocap hand.

## Build instructions

This is a standard Golang project, so you can build it with the following command:

```shell
go mod vendor
make
```

This will build the binary in the `_output` directory.

```shell
_output
├── platforms
│         └── linux
│             └── amd64
│                 ├── rfmocap-app
│                 ├── rfmocap-grpc_local
│                 └── rfmocap-serial_playground
├── tmp
└── tools
```

- `rfmoacp-app`: The main application
- `rfmoacp-grpc_local`: The gRPC test client
- `rfmoacp-serial_playground`: The serial port test client

## Deployment

This project supports two type of deployment:

- Docker based. This type is tested on x86_64 and arm64.
- Service based deployment. This type is ONLY test on Raspberry Pi 4 with Rasbian OS.

### Docker based deployment

To build the docker image, you can use the following command:

```shell
make image.build
```

This will build the docker image with the tag `docker.io/davidliyutong/rfmocap-app:latest`.

To run the docker image, you can use the following command:

```shell
cd manifests/docker-compose
docker compose up
```

This will start the docker container. To run the docker container in the background, you can use the following command:

```shell
cd manifests/docker-compose
docker compose up -d
```

> the docker compose will map /dev to the container, so the container can access the serial port.

### Service based deployment

To deploy the service, you can use the following command:

```shell
make service.install
```

> you might need to install `sudo` command

This will install the service to the system. You need to manually copy the config from `manifests/config/{left|right}/config.yaml` to `/etc/rfmocap-app/config.yaml`.

To start the service, you can use the following command:

```shell
sudo systemctl start rfmocap
```

### Time sync service

The time sync utility will update system time with `ntpdate` command. To deploy the time sync service, you can use the following command:

```shell
sudo apt install ntpdate # or use your own package manager
make time_sync.install NTP_SERVER=[YOUR_NTP_SERVER_IP]
```

## Developer's Guide

### Update gRPC code

Once you update the gRPC proto, you need to regenerate the gRPC code. You can use the following command:

```shell
make task.pb.server
make task.pb.python_client
```

This will update the server code at internal/pb and the python client code at ../api/python_client


