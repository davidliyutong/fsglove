ROOT_PACKAGE = $(shell pwd)
VERSION_PACKAGE = rfmocap-ng/pkg/version

_BINARY_PREFIX = rfmocap-
AUTHOR = davidliyutong

.PHONY: all
all: go.build

include scripts/make-rules/common.mk
include scripts/make-rules/golang.mk
include scripts/make-rules/docker.mk

define USAGE_OPTIONS
	N_SERVERS: number of servers to start
	NTP_SERVER: ntp server to sync time with
endef
export USAGE_OPTIONS

.PHONY: task.pb.server
task.pb.server:
	@echo "===========> Generating protobuf files"
	@cd internal/pb && protoc --go_out=. --go_opt=paths=source_relative \
		--go-grpc_out=. --go-grpc_opt=paths=source_relative \
		imu_packet.proto

.PHONY: task.pb.python_client
task.pb.python_client:
	@echo "===========> Generating protobuf files for python"
	@mkdir -p client/python
	@python -m grpc_tools.protoc -I./internal/pb \
	    --python_out=./client/python --grpc_python_out=./client/python ./internal/pb/imu_packet.proto

.PHONY: clean
clean:
	@echo "===========> Cleaning all build output"
	@-rm -vrf $(OUTPUT_DIR)

.PHONY: demo
demo:
	@cd manifests/docker-compose && docker-compose up

.PHONY: demo.stop
demo.stop:
	@cd manifests/docker-compose && docker-compose down

.PHONY: service.install
service.install:
	@echo "===========> Installing binary"
	@echo $(OUTPUT_DIR)/platforms/$(GOOS)/$(GOARCH)/$(_BINARY_PREFIX)app
	@sudo install $(OUTPUT_DIR)/platforms/$(GOOS)/$(GOARCH)/$(_BINARY_PREFIX)app /usr/local/bin/$(_BINARY_PREFIX)app
	@echo "===========> Installing service"
	@sudo cp -v manifests/services/rfmocap.service /etc/systemd/system/rfmocap.service
	@echo "===========> Installing configuration"
	@sudo mkdir -p /etc/rfmocap
	@echo "===========> Reloading service"
	@sudo systemctl daemon-reload
	@sudo systemctl enable rfmocap.service

.PHONY: service.uninstall
service.uninstall:
	@echo "===========> Uninstalling service"
	@sudo systemctl stop rfmocap.service
	@sudo systemctl disable rfmocap.service
	@sudo rm -v /etc/systemd/system/rfmocap.service
	@sudo systemctl daemon-reload
	@echo "===========> Uninstalling binary"
	@sudo rm -v /usr/local/bin/$(_BINARY_PREFIX)app

.PHONY: service.upgrade
service.upgrade:
	@echo "===========> Stopping service"
	@sudo systemctl stop rfmocap.service
	@echo "===========> Updating binary"
	@sudo install $(OUTPUT_DIR)/platforms/$(GOOS)/$(GOARCH)/$(_BINARY_PREFIX)app /usr/local/bin/$(_BINARY_PREFIX)app
	@echo "===========> Reloading service"
	@sudo systemctl daemon-reload

# TODO: add time_sync install/uninstall targets
.PHONY: time_sync.install
time_sync.install:
	@echo "===========> Installing ntp sync binary"
	@echo sudo apt install ntp
	@echo sudo systemctl restart ntpd
	@echo sudo systemctl enable ntpd
	@echo $(OUTPUT_DIR)/platforms/$(GOOS)/$(GOARCH)/$(_BINARY_PREFIX)time_sync
	@sudo install $(OUTPUT_DIR)/platforms/$(GOOS)/$(GOARCH)/$(_BINARY_PREFIX)time_sync /usr/local/bin/$(_BINARY_PREFIX)time_sync
	@echo "===========> Installing ntp sync service"
	@sudo cp -v manifests/services/$(_BINARY_PREFIX)time_sync.service /etc/systemd/system/$(_BINARY_PREFIX)time_sync.service
	@echo "===========> Installing ntp sync configuration"
	sudo sed -i 's/$${NTP_SERVER}/$(NTP_SERVER)/g' /etc/systemd/system/$(_BINARY_PREFIX)time_sync.service
	@echo "===========> Reloading service"
	@sudo systemctl daemon-reload
	@sudo systemctl enable $(_BINARY_PREFIX)time_sync.service

.PHONY: time_sync.uninstall
time_sync.uninstall:
	@echo "===========> Uninstalling ntp sync service"
	@sudo systemctl stop $(_BINARY_PREFIX)time_sync.service
	@sudo systemctl disable $(_BINARY_PREFIX)time_sync.service
	@sudo rm -v /etc/systemd/system/$(_BINARY_PREFIX)time_sync.service
	@sudo systemctl daemon-reload
	@echo "===========> Uninstalling binary"
	@sudo rm -v /usr/local/bin/$(_BINARY_PREFIX)time_sync
